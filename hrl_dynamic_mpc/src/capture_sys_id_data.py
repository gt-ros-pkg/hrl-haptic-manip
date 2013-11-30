#
#
# Copyright (c) 2013, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \authors: Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
# \adviser: Charles Kemp (Healthcare Robotics Lab, Georgia Tech.)


import darci_client as dc
import opt_traj_tools as ott
import cPickle as pkl
import time
import roslib; roslib.load_manifest('hrl_dynamic_mpc')
import rospy

file_name = './faster_left_arm_traj_2.pkl'
rospy.init_node( 'system_id_on_darci', anonymous = True )

if __name__ == "__main__":

    robot = dc.DarciClient(arm = 'l', record_data = True)
    
    data = pkl.load(open(file_name, 'r'))
    print "loaded pickle, sleeping now ..."
    time.sleep(3.)

    start = time.time()
    cur_time = time.time()

    count = 0

    #rate =rospy.Rate(100)
    rate =rospy.Rate(100)

    cmd_angles = []
    t = time.time()
    while time.time() - start < 200: #100
    #while count < 10000:
        #q, qd, qdd = ott.get_ref_traj_at_time(data['params'], count*0.015, 7, 5)
        #q, qd, qdd = ott.get_ref_traj_at_time(data['params'], count*0.03, 7, 5)
        q, qd, qdd = ott.get_ref_traj_at_time(data['params'], (time.time()-t), 7, 5)
        robot.setDesiredJointAngles(q)
        robot.updateSendCmd()
        cmd_angles.append(q)
        robot.recordCurData()

        rate.sleep()
        count = count + 1

    angle_data, angle_vel_data, torque_data, times = robot.getRecordedData()

    d = {}
    d['angle_data'] = angle_data
    d['angle_vel_data'] = angle_vel_data
    d['torque_data'] = torque_data
    d['times'] = times
    d['angle_cmds'] = cmd_angles
    
    f = open('./data_buffer.pkl', 'w')
    pkl.dump(d, f)
