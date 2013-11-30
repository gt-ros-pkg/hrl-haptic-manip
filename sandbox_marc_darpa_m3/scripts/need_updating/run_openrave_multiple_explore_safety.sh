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
    echo Use obstacles or not ...1 is including moveable obstacles, 0 is ignoring them
    exit
fi

if [ "$5" = "" ]; then
    echo Give starting moveable number
    exit
fi

if [ "$6" = "" ]; then
    echo Give ending moveable number
    exit
fi

if [ "$7" = "" ]; then
    echo Give starting fixed number
    exit
fi

if [ "$8" = "" ]; then
    echo Give ending fixed number
    exit
fi

PWD=`pwd`
cd $1

DARPA_M3_FOLDER=`rospack find darpa_m3`
N=$2
END=$3
M=$5
M_END=$6
F=$7
F_END=$8

while [ $F -le $F_END ] ;
do
    while [ $M -le $M_END ];
    do
	E=`printf "F%02d_M%02d" $F $M`
	cd $E
	echo $E
	while [ $N -le $END ];
	do
	    D=`printf "%04d" $N`
	    echo $D
	    mkdir $D
	    cd $D
	    if [ "$4" == 1 ]; then   
		if [ -f ./openrave_result_all_safety.pkl ]
		then
		    echo "openrave_result_all_safety.pkl already exists, skipping folder"
		else
		    python $DARPA_M3_FOLDER/src/geometric_search/planar_openrave.py --pkl=`ls reach_problem_dict*.pkl` --quiet --linkage three_link_planar --pkl_out='openrave_result_all.pkl'
		fi
	    else
		if [ -f ./openrave_result_ignore_moveable_safety.pkl ]
		then
		    echo "openrave_result_ignore_moveable_safety.pkl already exists, skipping folder"
		else
		    python $DARPA_M3_FOLDER/src/geometric_search/planar_openrave.py --pkl=`ls reach_problem_dict*.pkl` --quiet --ignore_moveable --pkl_out='openrave_result_ignore_moveable_safety.pkl' --linkage three_link_planar --add_safety 0.01
		fi
	    fi
	    cd ../
	    pwd
	    let N=$N+1
	done
	N=$2
	let M=$M+2
	cd ../
    done
    M=$5
    let F=$F+2
done

# while [ $F -le $F_END ];
# do
#     while[ $M -le $M_END ];
#     do
# 	echo Hi there
# 	# E=`printf "F%02d_M%02d" $F $M`
# 	# cd $E
# 	while [ $N -le $END ];
# 	do
# 	    echo hello
# 	    # D=`printf "%04d" $N`
# 	    # cd $D
# 	    # if [ "$4" = "1" ]; then   
# 	    # 	python $DARPA_M3_FOLDER/src/geometric_search/planar_openrave.py --pkl=`ls reach_problem_dict*.pkl` --quiet 
# 	    # else
# 	    # 	python $DARPA_M3_FOLDER/src/geometric_search/planar_openrave.py --pkl=`ls reach_problem_dict*.pkl` --quiet --ignore_moveable --pkl_out='openrave_result_ignore_moveable.pkl'
# 	    # fi

# 	    # cd ../
# 	    # let N=$N+1
# 	done
# 	N=$2
# 	let M=$M+2
# #	cd ../
#     done
#     M=$5
#     let F=$F+2
# done

cd $PWD




