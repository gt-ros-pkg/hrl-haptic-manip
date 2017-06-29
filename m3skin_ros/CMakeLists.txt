cmake_minimum_required(VERSION 2.8.3)
project(m3skin_ros)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs std_msgs geometry_msgs hrl_haptic_manipulation_in_clutter_msgs message_generation)

add_message_files(
  FILES
  RawTaxelArray.msg
)
#uncomment if you have defined services
add_service_files(
  FILES
  None_String.srv  
  None_TransformArray.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES geometry_msgs std_msgs hrl_haptic_manipulation_in_clutter_msgs
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS visualization_msgs std_msgs geometry_msgs hrl_haptic_manipulation_in_clutter_msgs
    INCLUDE_DIRS 
    LIBRARIES 
)