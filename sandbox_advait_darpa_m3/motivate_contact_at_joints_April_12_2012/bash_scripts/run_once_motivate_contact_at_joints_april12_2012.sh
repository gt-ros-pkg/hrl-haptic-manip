#!/bin/bash

if [ "$1" = "" ]; then
    echo Give name of reach_problem_dict
    exit
fi

if [ "$2" = "use_skin" ]; then
    CONTROLLER_SWITCH=''
elif [ "$2" = "ignore_skin" ]; then
    CONTROLLER_SWITCH=--is
else
    echo 'Specify either use_skin or ignore_skin'
    exit
fi

DIR_NAME=`echo $1 | awk -F"/" '{ print $NF }' | awk -F"." '{ print $1 }' | awk -F"_" '{ print $4"_"$5"_"$6 }'`

mkdir $DIR_NAME
cd $DIR_NAME

rosparam set use_sim_time true
rosparam delete /m3
rosparam delete /launch
rosparam set /m3/software_testbed/resolution 100

if [ -f ./overall_result.pkl ];
then
    echo "##########################################"
    echo "Result pkl exists. Ignoring this trial."
    echo "##########################################"
    exit
else
    echo $1
    rosrun hrl_common_code_darpa_m3 obstacles.py --pkl=$1

    rosrun hrl_software_simulation_darpa_m3 simulator /skin/contacts:=/skin/contacts_unused __name:=software_simulation_node &
    rosrun hrl_common_code_darpa_m3 taxel_array_to_skin_contact.py __name:=taxel_array_to_skin_contact &
    rosrun hrl_software_simulation_darpa_m3 sim_arm_param_upload.py --three_link_with_hand
    sleep 3

    rosrun sandbox_advait_darpa_m3 log_and_monitor_node.py --sim3_with_hand __name:=ode_log_and_monitor_node &
    rosrun sandbox_advait_darpa_m3 contact_memory.py --sim3_with_hand __name:=contact_memory_node &
    rosrun sandbox_advait_darpa_m3 switch_among_controllers_node.py --batch --sim3_with_hand --ignore_mobile_base $CONTROLLER_SWITCH
fi

rosnode kill software_simulation_node
rosnode kill taxel_array_to_skin_contact
rosnode kill ode_log_and_monitor_node
rosnode kill contact_memory_node



