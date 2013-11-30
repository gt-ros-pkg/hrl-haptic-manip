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
    echo Give starting moveable number
    exit
fi

if [ "$5" = "" ]; then
    echo Give ending moveable number
    exit
fi

if [ "$6" = "" ]; then
    echo Give starting fixed number
    exit
fi

if [ "$7" = "" ]; then
    echo Give ending fixed number
    exit
fi

if [ "$8" = "" ]; then
    echo Give resolution per meter
    exit
fi

PWD=`pwd`
cd $1

DARPA_M3_FOLDER=`rospack find darpa_m3`
N=$2
END=$3
M=$4
M_END=$5
F=$6
F_END=$7

SKIN_RESOLUTION=$8

while [ $F -le $F_END ] ;
do
    while [ $M -le $M_END ];
    do
	E=`printf "F%02d_M%02d" $F $M`
	cd $E
	echo $E

	while [ $N -le $END ]
	do
	    D=`printf "%04d" $N`
	    let N=$N+1
        $DARPA_M3_FOLDER/scripts/run_ode_once_new_aug_24.sh $D delta_qp_multiple_reach_oct17 $SKIN_RESOLUTION
  	    #$DARPA_M3_FOLDER/scripts/run_ode_once.sh $D delta_qp_jep_gen_20110719_max_force_5N_25Hz_reach_in_out_$SKIN_RESOLUTION $SKIN_RESOLUTION --opt_vmc
  	    #$DARPA_M3_FOLDER/scripts/run_ode_once.sh $D vmc_20110719_$SKIN_RESOLUTION $SKIN_RESOLUTION --opt_vmc
	    #$DARPA_M3_FOLDER/scripts/run_ode_once.sh $D
	done
	N=$2
	let M=$M+2
	cd ../
    done
    M=$4
    let F=$F+2
done

