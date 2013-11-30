#/bin/bash

#
# Use this script to generate 200 trials with 25 different obstacle
# configurations and 8 goal locations along a polar grid for each of
# the 25 obstacle locations.
#

N=0

while [ $N -lt 25 ] ;
do
    let N=$N+1
    rosrun hrl_common_code_darpa_m3 obstacles.py --fixed=40 --sliding=40 --xmin=0.2 --xmax=0.7 --ymin=-0.5 --ymax=0.5 --save_pkl
    NM=reach_problem_dict_`printf "%02d" $N`.pkl
    mv reach_problem_dict.pkl $NM
    rosrun hrl_common_code_darpa_m3 distribute_goal_on_a_grid.py --nr=3 --nt=3 --rmin=0.4 --rmax=0.6 --tmin=-30 --tmax=30 --pkl $NM --sf --sim3_with_hand
    rm -f $NM
done

mkdir problem_dict_plots
mv *.png problem_dict_plots



