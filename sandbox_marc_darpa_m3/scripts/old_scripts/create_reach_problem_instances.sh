#!/bin/bash -x


if [ "$1" = "" ]; then
    echo Give a top directory name.
    exit
fi

if [ "$2" = "" ]; then
    echo Give starting number of directory
    exit
fi


if [ "$3" = "" ]; then
    echo Give number of instances to create
    exit
fi

if [ "$4" = "" ]; then
    echo Give number of fixed obstacles.
    exit
fi

if [ "$5" = "" ]; then
    echo Give number of sliding obstacles
    exit
fi

if [ "$6" = "" ]; then
    echo Give number for batch, 0 otherwise
    exit
fi

N_INSTANCES=$3
N_FIXED=$4
N_SLIDE=$5

PWD=`pwd`
cd $1

DARPA_M3_FOLDER=`rospack find darpa_m3`

#OBS_ARGS="--fixed=$N_FIXED --sliding=$N_SLIDE --xmin=0.15 --xmax=0.6 --ymin=-0.3 --ymax=0.3 --save_pkl --check_openrave"
OBS_ARGS="--fixed=$N_FIXED --sliding=$N_SLIDE --xmin=0.2 --xmax=0.6 --ymin=-0.5 --ymax=0.2 --save_pkl --radius 0.01"

N=0
while [ $N -lt $N_INSTANCES ]
do
    let DIR_NUM=$2+$N
    D=`printf "%04d" $DIR_NUM`
    mkdir $D
    cd $D
    if [ -f ./reach_problem_dict.pkl ] && [ -f ./openrave_result_ignore_moveable.pkl ]
    then
	echo "openrave and config file exists."
    else
	rm *.pkl
	$DARPA_M3_FOLDER/src/software_simulation/obstacles.py $OBS_ARGS
	if [ "$6" = "0" ]
	then
	    python $DARPA_M3_FOLDER/src/geometric_search/planar_openrave.py --pkl=`ls reach_problem_dict*.pkl` --ignore_moveable --quiet --pkl_out='openrave_result_ignore_moveable.pkl' --linkage three_link_planar 
	else
	    python $DARPA_M3_FOLDER/src/geometric_search/planar_openrave.py --pkl=`ls reach_problem_dict*.pkl` --ignore_moveable --quiet --pkl_out='openrave_result_ignore_moveable.pkl' --linkage three_link_planar --batch $6
	fi

    fi
    #python obstacles.py --fixed=$N_FIXED --sliding=$N_SLIDE --xmin=0.15 --xmax=0.6 --ymin=-0.3 --ymax=0.3 --save_pkl

    # if [ "$?" = "1" ]; then
    #     echo No collision free path. Need to regenerate obstacles.
    #     let N=$N-1
    # fi

    let N=$N+1
    cd ..
done

cd $PWD
