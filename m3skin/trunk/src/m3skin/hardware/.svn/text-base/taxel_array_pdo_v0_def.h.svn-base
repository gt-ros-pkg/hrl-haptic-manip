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


#ifndef M3_TAXEL_ARRAY_PDO_V0_H
#define M3_TAXEL_ARRAY_PDO_V0_H

#include "m3rt/base/m3ec_def.h"

#ifdef __KERNEL__
#define int16_t short
#define int32_t int
#else
#ifndef EMBEDDED
#include <sys/types.h>
#include <stdint.h>
#endif
#endif

#define TAXEL_PER_BRANCH 12*16
#define NUM_BRANCHES 2
#define NUM_TAXEL TAXEL_PER_BRANCH*NUM_BRANCHES


/////////////////////////////// TAXEL_ARRAY_PDO_V0 /////////////////////////////////////////////////////

typedef struct 
{
	int16_t		config;					//Reserved	
} __attribute__((packed))  M3TaxelArrayPdoV0Cmd;


typedef struct 
{
	int16_t		debug;			//Reserved	
	int16_t		value[NUM_TAXEL];		//
}__attribute__((packed))  M3TaxelArrayPdoV0StatusExt;


#define M3TAX_PDO_V0_STATUS_EXT_SZ (MAX_PDO_ENTRY_SIZE*3)+14
//#define M3TAX_PDO_V0_STATUS_EXT_SZ 8
typedef struct 
{
	int16_t		ext_start_idx;
	unsigned char	ext[M3TAX_PDO_V0_STATUS_EXT_SZ];
}__attribute__((packed))  M3TaxelArrayPdoV0Status;



#endif
