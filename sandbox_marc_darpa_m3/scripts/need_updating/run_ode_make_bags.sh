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


#rm *skin_controller*
$DARPA_M3_FOLDER/src/software_simulation/obstacles.py --pkl=`ls reach_problem_dict*.pkl`
$DARPA_M3_FOLDER/bin/demo_kinematic /skin/contacts:=/skin/contacts_unused _include_mobile_base:=0 &
# $DARPA_M3_FOLDER/bin/demo_kinematic /skin/contacts:=/skin/contacts_unused &
rosrun darpa_m3 taxel_array_to_skin_contact.py &
$DARPA_M3_FOLDER/src/geometric_search/three_link_planar.py
$DARPA_M3_FOLDER/src/software_simulation/contact_logger.py $contact_logger_fname &
python $DARPA_M3_FOLDER/src/software_simulation/draw_bodies.py &
python $DARPA_M3_FOLDER/src/hardware_in_loop_simulation/viz_taxel_array.py &
sleep 5

python $DARPA_M3_FOLDER/src/sandbox_advait/contact_memory.py --sim &
python $DARPA_M3_FOLDER/src/sandbox_marc/haptic_controllers/ode_sim_guarded_move.py --batch $4 --fname=$controller_log_fname



rosnode kill draw_all_bodies 
rosnode kill sim_arm
rosnode kill taxel_array_to_skin_contact
rosnode kill ode_contact_logging
rosnode kill contact_memory_node
rosnode kill taxel_array_viz_publisher

# killall demo_kinematic
# killall taxel_array_to_skin_contact.py


cd $PWD


