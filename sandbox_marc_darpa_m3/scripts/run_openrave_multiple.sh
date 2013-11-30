#!/bin/bash -x

# usage: ./run_ode_multiple.sh <direc> <start trial num> <end trial num>

if [ "$1" = "" ]; then
    echo Give a top directory name.
    exit
fi

if [ "$2" = "" ]; then
    echo Give starting trial number.
    exit
fi

if [ "$3" = "" ]; then
    echo Give end trial number.
    exit
fi

if [ "$4" = "" ]; then
    echo Use obstacles or not ...0 is including moveable obstacles, 1 is ignoring them
    exit
fi



PWD=`pwd`
cd $1

DARPA_M3_FOLDER=`rospack find darpa_m3`
N=$2
END=$3

while [ $N -le $END ]
do
    D=`printf "%04d" $N`
    cd $D
    if [ "$4" = 0 ]; then   
	python $DARPA_M3_FOLDER/src/geometric_search/planar_openrave.py --pkl=`ls reach_problem_dict*.pkl` --quiet --linkage three_link_planar
    else
	python $DARPA_M3_FOLDER/src/geometric_search/planar_openrave.py --pkl=`ls reach_problem_dict*.pkl` --quiet --ignore_moveable --pkl_out='openrave_result_ignore_moveable.pkl' --linkage three_link_planar
    fi

    cd ..
    let N=$N+1
done


cd $PWD




