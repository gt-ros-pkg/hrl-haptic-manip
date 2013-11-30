/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <rtai_sched.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>

#include <rtai_nam2num.h>
#include <rtai_registry.h>

#include "ecrt.h"
	
#include "pthread.h"

#include "m3rt/base/m3ec_def.h"
#include "m3rt/base/m3ec_pdo_v0_def.h"
#include "m3rt/base/m3ec_pdo_v1_def.h"
#include "m3rt/base/m3ec_pdo_v2_def.h"
#include "m3rt/base/m3rt_def.h"
#include "../hardware/taxel_array_pdo_v0_def.h"
#include "stdio.h"


////////////////////////////////////////////////////////////////////////////////////
void SlaveEcShmPrettyPrint(M3EcSlaveShm * shm);
void SysEcShmPrettyPrint(M3EcSystemShm * shm);


void M3SkinPdoV0StatusPrettyPrint(M3TaxelArrayPdoV0Status * d,int sn);
void M3SkinPdoV0CmdPrettyPrint(M3TaxelArrayPdoV0Cmd * d,int sn);

////////////////////////////////////////////////////////////////////////////////////
static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static void endme(int dummy) { end=1; }
////////////////////////////////////////////////////////////////////////////////////

static inline void count2timeval(RTIME rt, struct timeval *t)
{
	t->tv_sec = rtai_ulldiv(count2nano(rt), 1000000000, (unsigned long *)&t->tv_usec);
	t->tv_usec /= 1000;
}

static void* rt_system_thread(void * arg)
{
	struct timeval tv;	
	int64_t ts1, ts2;
	SEM * shm_sem;
	SEM * sync_sem;
	RT_TASK *task;
	
	M3EcSystemShm * sys = (M3EcSystemShm *)arg;
	printf("Starting real-time thread\n",0);
	RTIME t_last;
	int cntr=0;
	task = rt_task_init_schmod(nam2num("M3SYSP"), 0, 0, 0, SCHED_FIFO, 0xF);
	rt_allow_nonroot_hrt();
	if (task==NULL)
	{
		printf("Failed to create RT-TASK M3SYSP\n",0);
		return 0;
	}
	shm_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3LSHM));
	if (!shm_sem)
	{
		printf("Unable to find the SEMNAM_M3LSHM semaphore.\n",0);
		rt_task_delete(task);
		return 0;
	}
	//else
	//	printf("Allocated shm_sem semaphore  %08x \n",shm_sem);
	
	sync_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3SYNC));
	if (!sync_sem)
	{
		printf("Unable to find the SEMNAM_M3SYNC semaphore.\n",0);
		rt_task_delete(task);
		rt_sem_delete(shm_sem);
		return 0;
	}
	//else
	//	printf("Allocated sync_sem semaphore  %08x \n",sync_sem);
	
	RTIME tick_period = nano2count(RT_TIMER_TICKS_NS); 
	RTIME now = rt_get_time();
	rt_task_make_periodic(task, now + tick_period, tick_period); 
	mlockall(MCL_CURRENT | MCL_FUTURE);
	rt_make_hard_real_time();
	t_last=now;
	sys_thread_active=1;
	uint64_t tl;
	while(!sys_thread_end)
	{
		rt_sem_wait(sync_sem);
		rt_sem_wait(shm_sem);
		if (cntr%200==0)
		{
			now=rt_get_time_ns();
			float dt = (now-t_last)/1000000.0;
			count2timeval(nano2count(rt_get_real_time_ns()), &tv);
			printf("\n\nM3 Cycle: %d: 200 cycles in %4.3f ms. EC cycles: %d\n", cntr,dt, sys->counter);
			printf("DT: timestamp_dt (uS) : %lld\n",(sys->timestamp_ns-tl)/1000);
			t_last=now;
			SysEcShmPrettyPrint(sys);
		}
		tl=sys->timestamp_ns;
		cntr++;
		rt_sem_signal(shm_sem);
		rt_task_wait_period();
	}	
	printf("Exiting RealTime Thread...\n",0);
	rt_make_soft_real_time();
	rt_task_delete(task);
	sys_thread_active=0;
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////
int main (void)
{	
	
	M3EcSystemShm * sys;
	RT_TASK *task;
	pthread_t ptsys;
	int cntr=0;
	
	signal(SIGINT, endme);

	sys = rtai_malloc (nam2num(SHMNAM_M3MKMD),1);
	if (sys==-1) 
	{
		printf("Error allocating shared memory\n");
		return 0;
	}
	int ns=sys->slaves_active;
	printf("Found %d active M3 EtherCAT slaves\n",ns);
	if (ns==0)
	{
		printf("No slaves available. Exiting...\n");
		return 0;
	}
	rt_allow_nonroot_hrt();
	if (!(task = rt_task_init_schmod(nam2num("M3MAIN"), RT_TASK_PRIORITY, 0, 0, SCHED_FIFO, 0xF)))
	{
		rt_shm_free(nam2num(SHMNAM_M3MKMD));
		printf("Cannot init the RTAI task %s\n","M3MAIN");
		return 0;
	}
	hst=rt_thread_create((void*)rt_system_thread, sys, 10000);
	usleep(100000); //Let start up
	if (!sys_thread_active)
	{
		rt_task_delete(task);
		rt_shm_free(nam2num(SHMNAM_M3MKMD));
		printf("Startup of thread failed.\n",0);
		return 0;
	}
	while(!end)
	{
		//SysEcShmPrettyPrint(sys);
		usleep(250000);
		
	}
	printf("Removing RT thread...\n",0);
	sys_thread_end=1;
	rt_thread_join(hst);
	if (sys_thread_active)printf("Real-time thread did not shutdown correctly\n");
	rt_task_delete(task);
	rt_shm_free(nam2num(SHMNAM_M3MKMD));
	return 0;
}

int64_t ts1_last=10000000;
int tcnt=0;
////////////////////////////////////////////////////////////////////////////////////
void SysEcShmPrettyPrint(M3EcSystemShm * shm)
{
	int64_t ts1, ts2;
	int i;
	printf("----- SysEcShm -----\n");
	printf("slaves_responding : %d\n",shm->slaves_responding );
	printf("slaves_active : %d\n",shm->slaves_active );
	printf("slaves_dropped : %d\n",shm->slaves_dropped );
	printf("link_up : %d\n",shm->link_up );
	printf("watchdog : %d\n",shm->watchdog );
	printf("timestamp_ns : %lld\n",shm->timestamp_ns );

	for (i=0;i<shm->slaves_responding;i++)
		if (shm->slave[i].active)
			SlaveEcShmPrettyPrint(&(shm->slave[i]));		       	
}

//////////////////////////LEGACY///////////////////////////////////////////////////////
void M3SkinPdoV0StatusPrettyPrint(M3TaxelArrayPdoV0Status * d,int sn)
{
	int i;
	printf("----- Status -----\n",0);
	printf("id %d: ext_start_idx: %d\n",sn,(int) d->ext_start_idx);
	for (i=0;i<M3TAX_PDO_V0_STATUS_EXT_SZ/2;i++)
	  printf("id %d: ext %d:  %d\n",sn,i,((int16_t*) d->ext)[i]);
}
///////////////////////////LEGACY/////////////////////////////////////////////////////
void M3SkinPdoV0CmdPrettyPrint(M3TaxelArrayPdoV0Cmd * d,int sn)
{
	printf("----- Command -----\n",0);		
	printf("sn %d: config: %d\n",sn,(int) d->config);
}



////////////////////////////////////////////////////////////////////////////////////
void SlaveEcShmPrettyPrint(M3EcSlaveShm * shm)
{
	printf("\n\n----------------- Slave: %d -----------------\n",shm->network_id);
	printf("active : %d\n",shm->active);
	printf("network_id : %d\n",shm->network_id);
	printf("serial_number : %d\n",shm->serial_number);
	printf("product_code : %d\n",shm->product_code);
	
	printf("online : %d\n",shm->online);
	printf("operational : %d\n",shm->operational);
	printf("al_state : %d\n",shm->al_state);
	
	////////////////////////////////////////
	if (shm->product_code==M3SKIN0_PRODUCT_CODE)
	{
		M3SkinPdoV0StatusPrettyPrint((M3TaxelArrayPdoV0Status *) shm->status,shm->serial_number);
		M3SkinPdoV0CmdPrettyPrint((M3TaxelArrayPdoV0Cmd *) shm->cmd,shm->serial_number);
	}

}
////////////////////////////////////////////////////////////////////////////////////
