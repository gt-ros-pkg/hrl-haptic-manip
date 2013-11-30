#!/bin/bash -x


if [ "$1" = "" ]; then
    echo Give a directory name.
    exit
fi

if [ "$2" = "" ]; then
    echo give option for controller, e.g. --opt_vmc.
    exit
fi

if [ "$3" = "" ]; then
    echo give skin resolution in taxels per meter.
    exit
fi

if [ "$4" = "" ]; then
    echo give alpha value percent of nominal stiffness
    exit
fi

# $4 is passed on to the ode_sim_guarded_move.py script. e.g.  --opt_force

PWD=`pwd`
cd $1
DARPA_M3_FOLDER=`rospack find darpa_m3`

roscore&
rosrun rviz rviz&
sleep 5
rosparam set use_sim_time true
rosparam delete /m3
rosparam delete /launch
roslaunch darpa_m3 ode_sim.launch &
sleep 5



if ! [ -f ./reach_problem_dict.pkl ];
then
    echo "there is no config file, skipping folder"
    exit
fi

$DARPA_M3_FOLDER/src/software_simulation/obstacles.py --pkl=`ls reach_problem_dict*.pkl`
#$DARPA_M3_FOLDER/bin/demo_kinematic /skin/contacts:=/skin/contacts_unused &
#rosrun darpa_m3 taxel_array_to_skin_contact.py &
python $DARPA_M3_FOLDER/src/forked_simulation_files/var_comp_optimize.py --pub_once_alpha $4
sleep 2
#$DARPA_M3_FOLDER/src/geometric_search/three_link_planar.py
#rosrun darpa_m3 draw_bodies.py &
python $DARPA_M3_FOLDER/src/forked_simulation_files/ode_sim_guarded_move.py $2

#killall ode_sim.launch
killall python /opt/ros/diamondback/ros/bin/roslaunch darpa_m3 ode_sim.launch rviz

cd $PWD


