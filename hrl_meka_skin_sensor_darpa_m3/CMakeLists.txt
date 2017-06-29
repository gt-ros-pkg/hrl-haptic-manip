cmake_minimum_required(VERSION 2.8.3)
project(hrl_meka_skin_sensor_darpa_m3)

find_package(catkin REQUIRED COMPONENTS rospy hrl_lib geometry_msgs hrl_msgs m3skin_ros hrl_haptic_manipulation_in_clutter_msgs)

catkin_python_setup()

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS hrl_lib geometry_msgs hrl_msgs m3skin_ros hrl_haptic_manipulation_in_clutter_msgs
    INCLUDE_DIRS 
    LIBRARIES 
)