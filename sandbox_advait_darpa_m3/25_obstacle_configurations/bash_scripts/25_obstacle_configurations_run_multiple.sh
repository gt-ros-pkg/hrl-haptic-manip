#!/bin/bash -x

if [ "$1" = "" ]; then
    echo Give directory with all the reach_problem_dict pkls
    exit
fi

if [ "$2" = "use_skin" ]; then
    echo 'mpc with skin'
elif [ "$2" = "use_ft_sensor" ]; then
    echo 'mpc with FT SENSOR'
elif [ "$2" = "ignore_skin" ]; then
    echo 'mpc WITHOUT skin'
else
    echo 'Specify either use_skin or use_ft_sensor or ignore_skin'
    exit
fi

if [ "$3" = "single_reach" ]; then
    echo 'SINGLE reach'
elif [ "$3" = "multiple_reaches" ]; then
    echo 'multiple reaches'
else
    echo 'Specify either single_reach or multiple_reaches'
    exit
fi

if [ "$4" = "" ]; then
    echo 'Specify an allowable force'
    exit
fi

find $1 -name "reach_problem_dict*.pkl" -exec `rospack find sandbox_advait_darpa_m3`/25_obstacle_configurations/bash_scripts/25_obstacle_configurations_run_once.sh {} $2 $3 $4 \;

python `rospack find sandbox_advait_darpa_m3`/25_obstacle_configurations/python_scripts/analyze_multiple_trial_results.py --dir=`pwd`



