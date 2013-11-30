#!/bin/bash


if [ "$1" = "" ]; then
    echo Give an experiment number
    exit
fi

if [ "$2" = "" ]; then
    echo "Specify a controller to run. {baseline, mpc}"
    exit
fi

mkdir $1

echo "Ctrl+C once the controller is done running"

# run the contact_memory and log_and_monitor_node ROS nodes.
python `rospack find sandbox_advait_darpa_m3`/src/mid_level_control/contact_memory.py --cody --arm_to_use=r &
python `rospack find sandbox_advait_darpa_m3`/src/mid_level_control/log_and_monitor_node.py --cody --arm_to_use=r &

# Now run the controller ROS node.
python mid_level_cinderblock_with_optitrak.py --use_$2

kill %
kill %

mv *.pkl $1

# now plot the histogram of forces.
python cody_cinderblock_visualize.py --dir=$1


