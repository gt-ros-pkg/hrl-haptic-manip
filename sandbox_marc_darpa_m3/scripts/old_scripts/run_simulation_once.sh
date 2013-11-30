#!/bin/bash -x

#usage ./run_simulation_once.sh <file path> <logging file path> [use_skin, use_ft_sensor, ignore_skin]
#                               [single_reach, multiple_reaches] [three_link_capsule, multi_link_X] <optional resolution>

if [ "$1" = "" ]; then
    echo 'need to give path to reach_problem_dict file ... EXITING'
    exit
fi

if [ "$2" = "" ]; then
    echo "need output log file prefix ... exiting"
    exit
fi

if [ "$3" = "use_skin" ]; then
    CONTROLLER_SWITCH=''
    SENSING="taxels"
elif [ "$3" = "use_ft_sensor" ]; then
    CONTROLLER_SWITCH=''
    SENSING="ft_at_base"
elif [ "$3" = "ignore_skin" ]; then
    CONTROLLER_SWITCH=--is
    SENSING="taxels"
else
    echo 'Specify either use_skin or use_ft_sensor or ignore_skin'
    exit
fi

if [ "$4" = "single_reach" ]; then
    REACH_SWITCH=--reach_greedy
elif [ "$4" = "multiple_reaches" ]; then
    REACH_SWITCH=--reach_mult
else
    echo 'Specify either single_reach or multiple_reaches'
    exit
fi

if [ "$5" = "three_link_capsule" ]; then
    LINK_TYPE=--planar_three_link_capsule
elif [ "$5" = "multi_link_three" ]; then
    LINK_TYPE=--multi_link_three_planar
elif [ "$5" = "multi_link_four" ]; then
    LINK_TYPE=--multi_link_four_planar
elif [ "$5" = "multi_link_five" ]; then
    LINK_TYPE=--multi_link_five_planar
elif [ "$5" = "multi_link_six" ]; then
    LINK_TYPE=--multi_link_six_planar
elif [ "$5" = "multi_link_seven" ]; then
    LINK_TYPE=--multi_link_seven_planar
elif [ "$5" = "multi_link_eight" ]; then
    LINK_TYPE=--multi_link_eight_planar
else
    echo 'Specify which type of linkage to use ... EXITING'
    exit
fi


if [ "$6" = "" ]; then
    echo "need to specify the taxel resolution"
    exit
else
    RESOLUTION=$6
fi

DIR_NAME=`echo $1 | awk -F"/" '{ print $NF }' | awk -F"." '{ print $1 }' | awk -F"_" '{ print $4"_"$5"_"$6 }'`

mkdir $DIR_NAME
cd $DIR_NAME

# $4 is passed on to the ode_sim_guarded_move.py script. e.g.  --opt_force

contact_logger_fname="$2"_"$5"_logger.pkl  #or should this be "$DIR_NAME""$2"_logger.pkl
controller_log_fname="$2"_"$5"_controller.pkl

rosparam set use_sim_time true
rosparam delete /m3
rosparam delete /launch
rosparam set /m3/software_testbed/resolution $6

#rosparam set /m3/software_testbed/stop_on_max false

if [ -f $contact_logger_fname ] && [ -f $controller_log_fname ]
then
    echo "##########################################"
    echo "SIMULATION RESULT FILE EXISTS... ignoring this trial ..."
    echo "##########################################"
    exit
else
    echo $1
    rosrun hrl_common_code_darpa_m3 obstacles.py --pkl=$1
    rosrun hrl_software_simulation_darpa_m3 sim_arm_param_upload.py $LINK_TYPE

    if [ "$SENSING" = "taxels" ]; then
        rosrun hrl_software_simulation_darpa_m3 simulator /skin/contacts:=/skin/contacts_unused __name:=software_simulation_node &
        rosrun hrl_common_code_darpa_m3 taxel_array_to_skin_contact.py __name:=taxel_array_to_skin_contact &
        sleep 3
        #rosrun sandbox_advait_darpa_m3 log_and_monitor_node.py --sim3 --disable __name:=ode_log_and_monitor_node &
        #rosrun sandbox_advait_darpa_m3 log_and_monitor_node.py --sim3 __name:=ode_log_and_monitor_node &

    elif [ "$SENSING" = "ft_at_base" ]; then
        rosrun hrl_software_simulation_darpa_m3 simulator /skin/contacts:=/skin/contacts_all /skin/taxel_array:=/skin/taxel_array_unused __name:=software_simulation_node &
        rosrun hrl_common_code_darpa_m3 skin_contact_to_resultant_force.py __name:=skin_contact_to_resultant_force &
        sleep 3
        #rosrun sandbox_advait_darpa_m3 log_and_monitor_node.py --sim3 --disable /skin/contacts:=/skin/contacts_all __name:=ode_log_and_monitor_node &
        #rosrun sandbox_advait_darpa_m3 log_and_monitor_node.py --sim3 /skin/contacts:=/skin/contacts_all __name:=ode_log_and_monitor_node &
    fi

    #rosrun sandbox_advait_darpa_m3 contact_memory.py --sim3 __name:=contact_memory_node &
    #rosrun sandbox_advait_darpa_m3 switch_among_controllers_node.py --batch --sim3 --ignore_mobile_base $CONTROLLER_SWITCH $REACH_SWITCH
    rosrun sandbox_marc_darpa_m3 contact_logger.py --fname=$contact_logger_fname --sim $LINK_TYPE &
    #rosrun sandbox_marc_darpa_m3 multi_contact_controller_init.py --batch --torque_vmc --ignore_mobile_base --sim --fname=$controller_log_fname $CONTROLLER_SWITCH $REACH_SWITCH $LINK_TYPE
    rosrun sandbox_marc_darpa_m3 multi_contact_controller_init.py --batch --qs_mpc_slow --ignore_mobile_base --sim --fname=$controller_log_fname $CONTROLLER_SWITCH $REACH_SWITCH $LINK_TYPE
fi

#sleep 10 

rosnode kill software_simulation_node
rosnode kill taxel_array_to_skin_contact
rosnode kill skin_contact_to_resultant_force
rosnode kill simulation_contact_logging



