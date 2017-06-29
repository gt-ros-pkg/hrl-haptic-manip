cmake_minimum_required(VERSION 2.8.3)
project(hrl_haptic_manipulation_in_clutter_srvs)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs hrl_haptic_manipulation_in_clutter_msgs hrl_msgs std_msgs message_generation)


add_service_files(
  FILES
  EnableHapticMPC.srv
  EnableService.srv
  GazeboObjectArray.srv  
  GazeboObjectState.srv  
  HapticMPCLogAndMonitorInfo.srv  
  HapticMPCLogging.srv  
  LogAndMonitorInfo.srv  
  ServiceBasedMPC.srv  
  ServiceLIC.srv  
  StartLog.srv
)

generate_messages(
    DEPENDENCIES geometry_msgs hrl_haptic_manipulation_in_clutter_msgs hrl_msgs std_msgs
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS geometry_msgs hrl_haptic_manipulation_in_clutter_msgs hrl_msgs std_msgs
    INCLUDE_DIRS 
    LIBRARIES 
)