#!/bin/bash -x

if [ "$1" = "" ]; then
    echo 'Give directory with all the reach_problem_dict pkls'
    exit
fi

if [ "$2" = "" ]; then
    echo 'Give prefix title to output pkl files'
    exit
fi


if [ "$3" = "use_skin" ]; then
    echo 'mpc with skin'
elif [ "$3" = "use_ft_sensor" ]; then
    echo 'mpc with FT SENSOR'
elif [ "$3" = "ignore_skin" ]; then
    echo 'mpc WITHOUT skin'
else
    echo 'Specify either use_skin or use_ft_sensor or ignore_skin'
    exit
fi

if [ "$4" = "single_reach" ]; then
    echo 'SINGLE reach'
elif [ "$4" = "multiple_reaches" ]; then
    echo 'multiple reaches'
else
    echo 'Specify either single_reach or multiple_reaches'
    exit
fi

if [ "$5" = "" ]; then
    echo 'Need the config name for linkage type ... exiting '
    exit
fi

if [ "$6" = "" ]; then
    echo 'Need the spatial resolution for taxels ... exiting '
    exit
fi


find $1 -name "reach_problem_dict*.pkl"  -exec `rospack find sandbox_marc_darpa_m3`/scripts/run_simulation_once.sh {} $2 $3 $4 $5 $6 \;

#cd <directory that you want output files in>
#then run the following for example

# ~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh
# ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_config_movable40_fixed120 
# qs_slow_mpc 
# use_skin 
# single_reach 
# multi_link_seven
# 100

#python `rospack find sandbox_advait_darpa_m3`/25_obstacle_configurations/python_scripts/analyze_multiple_trial_results.py --dir=`pwd`



