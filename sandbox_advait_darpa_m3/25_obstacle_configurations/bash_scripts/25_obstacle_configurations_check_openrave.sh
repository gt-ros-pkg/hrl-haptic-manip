#!/bin/bash -x

if [ "$1" = "" ]; then
    echo Give directory with all the reach_problem_dict pkls
    exit
fi

if [ "$2" = "" ]; then
    echo Give safety margin
    exit
fi

if [ "$3" = "" ]; then
    echo Give stopping distance
    exit
fi

if [ "$4" = "" ]; then
    echo Give n_theta
    exit
fi

if [ "$5" = "" ]; then
    echo Give n_rad
    exit
fi


find $1 -name "reach_problem_dict*.pkl" -exec python `rospack find sandbox_advait_darpa_m3`/25_obstacle_configurations/python_scripts/check_openrave.py --rpd {} --im --sp --safety_margin $2 --stopping_dist=$3 --n_theta=$4 --n_rad=$5 \;

mkdir plots
mv *.png plots

python `rospack find sandbox_advait_darpa_m3`/25_obstacle_configurations/python_scripts/compute_openrave_statistics.py --dir=`pwd`


