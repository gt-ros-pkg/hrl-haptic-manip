#!/bin/bash -x

if [ "$1" = "" ]; then
    echo Give directory with all the reach_problem_dict pkls
    exit
fi

if [ "$2" = "use_skin" ]; then
    echo 'mpc with skin'
elif [ "$2" = "ignore_skin" ]; then
    echo 'mpc WITHOUT skin'
else
    echo 'Specify either use_skin or ignore_skin'
    exit
fi

find $1 -name "reach_problem_dict*.pkl" -exec `rospack find sandbox_advait_darpa_m3`/motivate_contact_at_joints_April_12_2012/bash_scripts/run_once_motivate_contact_at_joints_april12_2012.sh {} $2 \;



