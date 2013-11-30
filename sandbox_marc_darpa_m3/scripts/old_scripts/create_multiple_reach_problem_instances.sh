#!/bin/bash -x

# usage: ./run_ode_multiple.sh <direc> <start trial num> <end trial num>

if [ "$1" = "" ]; then
    echo Give a top directory name.
    exit
fi

if [ "$2" = "" ]; then
    echo give starting number of directory
    exit
fi

if [ "$3" = "" ]; then
    echo Give number of instances to create
    exit
fi

if [ "$4" = "" ]; then
    echo Give start of desired fixed obstacles
    exit
fi

if [ "$5" = "" ]; then
    echo Give end of desired fixed obstacles
    exit
fi


if [ "$6" = "" ]; then
    echo Give start of desired moveable obstacles
    exit
fi

if [ "$7" = "" ]; then
    echo Give end of desired moveable obstacles
    exit
fi


PWD=`pwd`
cd $1

DARPA_M3_FOLDER=`rospack find darpa_m3`
N=$2
F_END=$5
M_END=$7

M=$6
F=$4
while [ $F -le $F_END ];
do
    while [ $M -le $M_END ];
    do
	E=`printf "F%02d_M%02d" $F $M`
	mkdir $E
	$DARPA_M3_FOLDER/scripts/create_reach_problem_instances.sh $E $2 $3 $F $M
	let M=$M+2
    done
    M=$6
    let F=$F+2
done







