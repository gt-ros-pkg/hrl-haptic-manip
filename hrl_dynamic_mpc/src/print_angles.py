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

import roslib
roslib.load_manifest('hrl_dynamic_mpc')
import hrl_lib.util as ut
import numpy as np
import os
from hrl_dynamic_mpc.srv import LogData
import threading
import rospy
import signal
import darci_client as dc

if __name__ == '__main__':

    rospy.init_node('printing_angles_and_ee')
    robot =  dc.DarciClient() 

    inp = None

    raw_input('press enter when ready to begin ..')

    q_data = {}
    ee_data = {}

    q_buf = []
    ee_buf = []
    while inp != 'q':
        raw_input('press enter to grab current angle and print position ... \n')
        robot.updateHapticState()
        joints = robot.joint_angles
        ee_position = robot.end_effector_position.reshape(3,1)
        
        q_buf.append(joints)
        ee_buf.append(ee_position)
        
        inp = raw_input('press q to quit\n enter to grab next angle or type name to move to next set:\n')
        if inp != None and inp != 'q' and inp!='':
            q_data[inp] = q_buf
            ee_data[inp] = ee_buf
            q_buf = []
            ee_buf = []

    data = {'q_configs':q_data, 'ee_positions':ee_data}
    ut.save_pickle(data, './joint_and_ee_data.pkl')
