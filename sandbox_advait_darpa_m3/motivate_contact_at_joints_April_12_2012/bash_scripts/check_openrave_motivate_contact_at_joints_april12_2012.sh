#!/bin/bash -x

if [ "$1" = "" ]; then
    echo Give directory with all the reach_problem_dict pkls
    exit
fi

if [ "$2" = "" ]; then
    echo Give safety margin
    exit
fi


find $1 -name "reach_problem_dict*.pkl" -exec python `rospack find sandbox_advait_darpa_m3`/25_obstacle_configurations/python_scripts/check_openrave.py --rpd {} --im --sp --safety_margin $2 \;

mkdir plots
mv *.png plots



