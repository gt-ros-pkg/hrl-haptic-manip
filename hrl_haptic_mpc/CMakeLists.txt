cmake_minimum_required(VERSION 2.8.3)
project(hrl_haptic_mpc)
find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS rospy std_msgs geometry_msgs trajectory_msgs interactive_markers hrl_lib hrl_haptic_manipulation_in_clutter_msgs hrl_haptic_manipulation_in_clutter_srvs pykdl_utils pr2_controllers_msgs hrl_cody_arms moveit_msgs
    INCLUDE_DIRS
    LIBRARIES
)

catkin_install_python(PROGRAMS
                      scripts/haptic_mpc
                      scripts/mpc_teleop_rviz
                      scripts/mpc_teleop_rviz_head
                      scripts/mpc_reset
                      scripts/waypoint_generator
                      scripts/robot_haptic_state_node
                      scripts/runstop_safe_restart
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)

install(DIRECTORY viz_config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/viz_config)
