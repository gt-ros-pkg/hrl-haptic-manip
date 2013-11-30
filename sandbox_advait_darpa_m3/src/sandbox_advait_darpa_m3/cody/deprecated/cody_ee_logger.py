

#
# trying to figure out what sort of error I have if I put a marker on
# the end effector and track using optitrak vs using fwd kinemetics.
#


import roslib; roslib.load_manifest('darpa_m3')
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import numpy as np, math


def torso_to_world(p):
    trans, quat = tf_lstnr.lookupTransform('/world', '/torso_lift_link', rospy.Time(0))
    rot = tr.quaternion_to_matrix(quat)
    trans = np.matrix(trans).reshape(3,1)
    p_world = trans + rot * p
    return p_world

def ee_from_optitrak():
    trans, quat = tf_lstnr.lookupTransform('/world', '/fixed_obstacle_trackable', rospy.Time(0))
    trans = np.matrix(trans).reshape(3,1)
    return trans

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--log', action='store_true', dest='log',
                 help='log ee pos using encoders and optitrak.')
    p.add_option('--plot', action='store_true', dest='plot',
                 help='plot the result.')
    p.add_option('--pkl', action='store', dest='pkl', type='string',
                 help='pkl name.')
    opt, args = p.parse_args()

    if opt.log:
        import rospy
        import cody_arms.arm_client as cac
        import cody_arms.arms as ca
        import tf

        rospy.init_node('cody_ee_logger')

        tf_lstnr = tf.TransformListener()
        arms = ca.M3HrlRobot(end_effector_length = 0.16) # for tube.
        cody_arms = cac.MekaArmClient(arms)

        r_arm, l_arm = 'right_arm', 'left_arm'
        arm = r_arm

        raw_input('Hit ENTER to start logging.')

        for i in range(10):
            jep = cody_arms.get_jep(r_arm)
            rospy.sleep(0.1)


        rt = rospy.Rate(20)
        ee_world_l = []
        ee_optitrak_l = []
        while not rospy.is_shutdown():
            ee_pos, _ = cody_arms.end_effector_pos(arm)
            ee_world = torso_to_world(ee_pos)
            ee_optitrak = ee_from_optitrak()
            ee_world_l.append(ee_world)
            ee_optitrak_l.append(ee_optitrak)
            rospy.loginfo('got data')
            rt.sleep()

        d = {}
        d['encoders'] = ee_world_l
        d['optitrak'] = ee_optitrak_l
        ut.save_pickle(d, 'cody_ee_'+ut.formatted_time()+'.pkl')
    
    if opt.plot:
        import matplotlib.pyplot as pp
        import hrl_lib.matplotlib_util as mpu

        d = ut.load_pickle(opt.pkl)
        ee_enc_mat = np.column_stack(d['encoders'])
        ee_optitrak_mat = np.column_stack(d['optitrak'])
        err = ee_optitrak_mat - ee_enc_mat
        pp.plot(err[0,:].A1, err[1,:].A1, '.')
        pp.axis('equal')
        pp.show()






