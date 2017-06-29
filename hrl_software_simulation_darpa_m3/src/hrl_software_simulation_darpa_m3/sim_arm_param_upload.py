#!/usr/bin/env python
import roslib; roslib.load_manifest('hrl_software_simulation_darpa_m3')
import rospy
import sys


# mod is short for module
def upload_to_param_server(mod):
    rospy.set_param('m3/software_testbed/joints/axes', mod.b_jts['axis'])
    rospy.set_param('m3/software_testbed/joints/anchor', mod.b_jts['anchor'])
    rospy.set_param('m3/software_testbed/joints/max', mod.b_jts['jt_lim_max'])
    rospy.set_param('m3/software_testbed/joints/min', mod.b_jts['jt_lim_min'])
    rospy.set_param('m3/software_testbed/joints/attach', mod.b_jts['jt_attach'])
    rospy.set_param('m3/software_testbed/joints/init_angle', mod.b_jts['jt_init'])
    rospy.set_param('m3/software_testbed/joints/num_joints', len(mod.b_jts['jt_lim_min']))
    rospy.set_param('m3/software_testbed/joints/imped_params_stiffness', mod.b_jts['jt_stiffness'])
    rospy.set_param('m3/software_testbed/joints/imped_params_damping', mod.b_jts['jt_damping'])
    rospy.set_param('m3/software_testbed/joints/types', mod.b_jts['jt_type'])
    rospy.set_param('m3/software_testbed/linkage/colors', mod.bodies['color'])
    rospy.set_param('m3/software_testbed/linkage/dimensions', mod.bodies['dim'])
    rospy.set_param('m3/software_testbed/linkage/mass', mod.bodies['mass'])
    rospy.set_param('m3/software_testbed/linkage/positions', mod.bodies['com_pos'])
    rospy.set_param('m3/software_testbed/linkage/shapes', mod.bodies['shapes'])
    rospy.set_param('m3/software_testbed/linkage/rotations', mod.bodies['rotation'])
#    rospy.set_param('m3/software_testbed/linkage/frames', mod.bodies['frames'])
    rospy.set_param('m3/software_testbed/linkage/names', mod.bodies['name'])

    rospy.set_param('m3/software_testbed/joints/imped_params_base_stiffness', mod.b_jts['base_jt_stiffness'])
    rospy.set_param('m3/software_testbed/joints/imped_params_base_damping', mod.b_jts['base_jt_damping'])
    rospy.set_param('m3/software_testbed/joints/base_max', mod.b_jts['base_jt_lim_max'])
    rospy.set_param('m3/software_testbed/joints/base_min', mod.b_jts['base_jt_lim_min'])
    rospy.set_param('m3/software_testbed/linkage/base_rotations', mod.bodies['base_rotation'])
    
    # this should always be the last parameter to get uploaded.
    rospy.set_param('m3/software_testbed/linkage/num_links', mod.bodies['num_links'])


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--planar_three_link_cuboid', action='store_true',
                 dest='pt_cuboid')
    p.add_option('--three_link_with_hand', action='store_true',
                 dest='tl_with_hand')
    p.add_option('--planar_three_link_capsule', action='store_true',
                 dest='pt_capsule', help='this specifies a 3 link capsule robot')
    p.add_option('--planar_three_link_capsule_nolim', action='store_true',
                 dest='pt_capsule_nolim', help='this specifies a 3 link capsule robot with broad joint limit')  
    p.add_option('--planar_three_link_capsule_mobile', action='store_true',
                 dest='pt_capsule_mobile', help='this specifies a 3 link capsule robot with a mobile base')
    
    p.add_option('--multi_link_one_planar', action='store_true',
                 dest='p_1_link', help='this specifies a 1 link capsule robot with equal link length')
    p.add_option('--multi_link_three_planar', action='store_true',
                 dest='p_3_link', help='this specifies a 3 link capsule robot with equal link length')
    p.add_option('--multi_link_four_planar', action='store_true',
                 dest='p_4_link', help='this specifies a 4 link capsule robot')
    p.add_option('--multi_link_five_planar', action='store_true',
                 dest='p_5_link', help='this specifies a 5 link capsule robot')
    p.add_option('--multi_link_six_planar', action='store_true',
                 dest='p_6_link', help='this specifies a 6 link capsule robot')
    p.add_option('--multi_link_seven_planar', action='store_true',
                 dest='p_7_link', help='this specifies a 7 link capsule robot')
    p.add_option('--multi_link_eight_planar', action='store_true',
                 dest='p_8_link', help='this specifies a 8 link capsule robot')

    p.add_option('--cody', action='store_true',
                 dest='cody', help='this specifies a 7 link capsule robot')
    p.add_option('--three_link_planar_cody', action='store_true',
                 dest='tl_p_cody', help='this specifies a 3 link planar cody robot')

    opt, args = p.parse_args()

    if opt.pt_cuboid:
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_cuboid as mod

    if opt.pt_capsule:
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as mod

    if opt.pt_capsule_nolim:
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule_nolim as mod        

    if opt.pt_capsule_mobile:
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule_mobile as mod
        
    if opt.tl_with_hand:
        import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as mod

    if opt.p_1_link:
        import hrl_common_code_darpa_m3.robot_config.multi_link_one_planar as mod

    if opt.p_3_link:
        import hrl_common_code_darpa_m3.robot_config.multi_link_three_planar as mod

    if opt.p_4_link:
        import hrl_common_code_darpa_m3.robot_config.multi_link_four_planar as mod

    if opt.p_5_link:
        import hrl_common_code_darpa_m3.robot_config.multi_link_five_planar as mod

    if opt.p_6_link:
        import hrl_common_code_darpa_m3.robot_config.multi_link_six_planar as mod

    if opt.p_7_link:
        import hrl_common_code_darpa_m3.robot_config.multi_link_seven_planar as mod

    if opt.p_8_link:
        import hrl_common_code_darpa_m3.robot_config.multi_link_eight_planar as mod

    if opt.cody:
        import hrl_common_code_darpa_m3.robot_config.cody_config_test as mod

    if opt.tl_p_cody:
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_cody as mod


    upload_to_param_server(mod)

