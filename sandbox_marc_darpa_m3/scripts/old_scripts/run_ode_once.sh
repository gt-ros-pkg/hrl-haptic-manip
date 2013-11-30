#!/bin/bash -x


if [ "$1" = "" ]; then
    echo Give a directory name.
    exit
fi

if [ "$2" = "" ]; then
    echo output log file name.
    exit
fi

if [ "$3" = "" ]; then
    echo give skin resolution in taxels per meter.
    exit
fi

# $4 is passed on to the ode_sim_guarded_move.py script. e.g.  --opt_force

contact_logger_fname="$2"_logger.pkl
controller_log_fname="$2"_controller.pkl

PWD=`pwd`
cd $1
DARPA_M3_FOLDER=`rospack find darpa_m3`

rosparam set use_sim_time true
rosparam delete /m3
rosparam delete /launch
rosparam set /m3/software_testbed/resolution $3
rosparam set /m3/software_testbed/stop_on_max false

if ! [ -f ./reach_problem_dict.pkl ];
then
    echo "there is no config file, skipping folder"
    exit
fi


if [ -f ./$contact_logger_fname ] && [ -f ./$controller_log_fname ]
then
    echo "ODE RESULT FILE EXISTS."
    exit
else
    #rm *skin_controller*
    $DARPA_M3_FOLDER/bin/demo_kinematic /skin/contacts:=/skin/contacts_unused _include_mobile_base:=0 &
    sleep 4
    $DARPA_M3_FOLDER/src/software_simulation/obstacles.py --pkl=`ls reach_problem_dict*.pkl`
    # $DARPA_M3_FOLDER/bin/demo_kinematic /skin/contacts:=/skin/contacts_unused &
    rosrun darpa_m3 taxel_array_to_skin_contact.py &
    $DARPA_M3_FOLDER/src/geometric_search/three_link_planar.py
    $DARPA_M3_FOLDER/src/software_simulation/contact_logger.py $contact_logger_fname &
    sleep 5

    python $DARPA_M3_FOLDER/src/sandbox_advait/contact_memory.py --sim &
    python $DARPA_M3_FOLDER/src/sandbox_marc/haptic_controllers/ode_sim_guarded_move.py --batch $4 --fname=$controller_log_fname --sim --reach_greedy #--reach_mult for other tests

fi


# killall demo_kinematic
# killall taxel_array_to_skin_contact.py
rosnode kill sim_arm
rosnode kill taxel_array_to_skin_contact
rosnode kill ode_contact_logging
rosnode kill contact_memory_node


cd $PWD


