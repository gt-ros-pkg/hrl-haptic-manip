#!/bin/bash

if [ "$1" = "" ]; then
    echo Give name of reach_problem_dict
    exit
fi

if [ "$2" = "use_skin" ]; then
    CONTROLLER_SWITCH=''
    SENSING="taxels"
elif [ "$2" = "use_ft_sensor" ]; then
    CONTROLLER_SWITCH=''
    SENSING="ft_at_base"
elif [ "$2" = "ignore_skin" ]; then
    CONTROLLER_SWITCH=--is
    SENSING="taxels"
else
    echo 'Specify either use_skin or use_ft_sensor or ignore_skin'
    exit
fi

if [ "$3" = "single_reach" ]; then
    REACH_SWITCH=--single_reach
elif [ "$3" = "multiple_reaches" ]; then
    REACH_SWITCH=''
else
    echo 'Specify either single_reach or multiple_reaches'
    exit
fi

if [ "$4" = "" ]; then
    echo 'Specify an allowable force'
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

    rosrun hrl_software_simulation_darpa_m3 sim_arm_param_upload.py --planar_three_link_capsule
    #rosrun hrl_software_simulation_darpa_m3 sim_arm_param_upload.py --planar_three_link_cuboid

    if [ "$SENSING" = "taxels" ]; then
        rosrun hrl_software_simulation_darpa_m3 simulator /skin/contacts:=/skin/contacts_unused __name:=software_simulation_node &
        rosrun hrl_common_code_darpa_m3 taxel_array_to_skin_contact.py __name:=taxel_array_to_skin_contact &
        sleep 3
        #rosrun sandbox_advait_darpa_m3 log_and_monitor_node.py --sim3 --disable __name:=ode_log_and_monitor_node &
        rosrun sandbox_advait_darpa_m3 log_and_monitor_node.py --sim3 __name:=ode_log_and_monitor_node &
    elif [ "$SENSING" = "ft_at_base" ]; then
        rosrun hrl_software_simulation_darpa_m3 simulator /skin/contacts:=/skin/contacts_all /skin/taxel_array:=/skin/taxel_array_unused __name:=software_simulation_node &
        rosrun hrl_common_code_darpa_m3 skin_contact_to_resultant_force.py __name:=skin_contact_to_resultant_force &
        sleep 3
        #rosrun sandbox_advait_darpa_m3 log_and_monitor_node.py --sim3 --disable /skin/contacts:=/skin/contacts_all __name:=ode_log_and_monitor_node &
        rosrun sandbox_advait_darpa_m3 log_and_monitor_node.py --sim3 /skin/contacts:=/skin/contacts_all __name:=ode_log_and_monitor_node &
    fi

    rosrun sandbox_advait_darpa_m3 contact_memory.py --sim3 __name:=contact_memory_node &
    rosrun sandbox_advait_darpa_m3 switch_among_controllers_node.py --batch --sim3 --ignore_mobile_base $CONTROLLER_SWITCH $REACH_SWITCH --acf=$4
fi

rosnode kill software_simulation_node
rosnode kill taxel_array_to_skin_contact
rosnode kill skin_contact_to_resultant_force
rosnode kill ode_log_and_monitor_node
rosnode kill contact_memory_node



