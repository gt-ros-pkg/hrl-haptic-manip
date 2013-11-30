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
    echo Give skin resolution
    exit
fi

PWD=`pwd`
cd $1

DARPA_M3_FOLDER=`rospack find darpa_m3`
N=$2
END=$3

SKIN_RESOLUTION=$4

#date +%H:%M:%S > start.txt
while [ $N -le $END ]
do
    D=`printf "%04d" $N`
    let N=$N+1
    #$DARPA_M3_FOLDER/scripts/run_ode_once.sh $D simplified_delta_qp_jep_gen_20111212_max_force_10N_$SKIN_RESOLUTION $SKIN_RESOLUTION --opt_qp_jep
    $DARPA_M3_FOLDER/scripts/run_ode_once.sh $D torque_vmc_20111212_max_force_5N_$SKIN_RESOLUTION $SKIN_RESOLUTION --opt_vmc #--opt_qp_jep
    #$DARPA_M3_FOLDER/scripts/run_ode_once_new_aug_24.sh $D delta_qp_multiple_reach $SKIN_RESOLUTION

done

#date +%H:%M:%S > end.txt






