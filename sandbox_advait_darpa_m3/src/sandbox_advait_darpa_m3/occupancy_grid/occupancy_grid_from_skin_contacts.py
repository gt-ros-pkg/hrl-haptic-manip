#!/usr/bin/python

import numpy as np, math
import sys

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy

from std_msgs.msg import Empty
from hrl_srvs.srv import FloatArray_None



class SkinToOccupancyGrid():
    def __init__(self, skin_client):
        self.sc = skin_client
        self.viz_cmd_pub = rospy.Publisher('/occupancy_grid_node/cmd/viz_simple', Empty)
        self.add_pts_srv = rospy.ServiceProxy('/occupancy_grid_node/srv/add_points_unstamped', FloatArray_None)

    ## Add points to the occupancy grid.
    # pts - 3xN np matrix or array
    def add_pts(self, pts):
        self.add_pts_srv(pts.T.A1)

    def step(self):
        f_l, _, loc_l, _ = self.sc.force_normal_loc_joint_list(True)
        f_mag_l = [np.linalg.norm(f) for f in f_l]

        pts_l = []
        for i in range(len(f_mag_l)):
            f_mag = f_mag_l[i]
            loc = loc_l[i]
            if f_mag > 5.:
                pts_l.append(loc)

        if pts_l != []:
            self.add_pts(np.column_stack(pts_l))
            #self.viz_cmd_pub.publish(Empty())



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--cody', action='store_true', dest='cody',
                 help='run on cody')
    p.add_option('--hil', action='store_true', dest='hil',
                 help='hardware-in-loop simulation with Cody')
    p.add_option('--sim', action='store_true', dest='sim',
                 help='run in simulation')
    opt, args = p.parse_args()

    if opt.sim or opt.hil:
        skin_topic_list = ['/skin/contacts']
    else:
        skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_ft']

    if opt.cody:
        from sandbox_advait_darpa_m3.cody.cody_guarded_move import Cody_SkinClient
        import hrl_cody_arms.cody_arm_client as cac

        rospy.init_node('cody_occupancy_grid_py')
        scl = Cody_SkinClient(skin_topic_list)

    elif opt.sim:
        from hrl_software_simulation_darpa_m3.ode_sim_guarded_move import ode_SkinClient

        rospy.init_node('ode_occupancy_grid_py')
        scl = ode_SkinClient(skin_topic_list)

    else:
        print 'Please specify a testbed within which to make an occupancy grid.'
        print 'Exiting ...'
        sys.exit()

    stog = SkinToOccupancyGrid(scl)
    
    rospy.sleep(1.)

    rt = rospy.Rate(100)
    rospy.loginfo('started occupancy grid from skin contact')
    ctr = 0
    while not rospy.is_shutdown():
        stog.step()
        rt.sleep()
        ctr += 1
        if ctr == 10:
            stog.viz_cmd_pub.publish(Empty())
            ctr = 0



