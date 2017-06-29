cmake_minimum_required(VERSION 2.8.3)
project(hrl_dynamic_mpc)

find_package(catkin REQUIRED COMPONENTS geometry_msgs hrl_lib sensor_msgs equilibrium_point_control hrl_cody_arms hrl_haptic_manipulation_in_clutter_msgs hrl_haptic_manipulation_in_clutter_srvs hrl_common_code_darpa_m3 m3skin_ros)

catkin_python_setup()

add_service_files(
  FILES
  LogData.srv
)

generate_messages(
#    DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS geometry_msgs hrl_lib sensor_msgs equilibrium_point_control hrl_cody_arms hrl_haptic_manipulation_in_clutter_msgs hrl_haptic_manipulation_in_clutter_srvs hrl_common_code_darpa_m3 m3skin_ros
    INCLUDE_DIRS 
    LIBRARIES 
)