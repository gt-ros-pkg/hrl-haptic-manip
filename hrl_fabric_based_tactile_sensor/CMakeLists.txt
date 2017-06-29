
cmake_minimum_required(VERSION 2.8.3)
project(hrl_fabric_based_tactile_sensor)

find_package(catkin REQUIRED COMPONENTS hrl_meka_skin_sensor_darpa_m3 m3skin_ros m3skin_viz)

catkin_python_setup()

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS hrl_meka_skin_sensor_darpa_m3 m3skin_ros m3skin_viz hrl_common_code_darpa_m3
    INCLUDE_DIRS 
    LIBRARIES 
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

