cmake_minimum_required(VERSION 2.8.3)
project(hrl_common_code_darpa_m3)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS rospy hrl_lib hrl_haptic_manipulation_in_clutter_msgs visualization_msgs m3skin_ros pykdl_utils
    INCLUDE_DIRS 
    LIBRARIES 
)

catkin_install_python(
  PROGRAMS scripts/viz
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)