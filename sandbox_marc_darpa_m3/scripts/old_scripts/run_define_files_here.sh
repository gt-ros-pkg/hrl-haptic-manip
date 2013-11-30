#!/bin/bash -x

#cd ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_config_movable40_fixed120 

NUM_LINKS="five"
FOLDER="5_links_validate/"

#############
#TEST 1
#############

cd ~/Desktop/$FOLDER

DIR1="75_obstacle_config_movable20_fixed20"
mkdir $DIR1
cd $DIR1

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR1 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100


#############
#TEST 2
#############
cd ~/Desktop/$FOLDER

DIR2="75_obstacle_config_movable80_fixed80"
mkdir $DIR2
cd $DIR2

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR2 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100

#############
#TEST 3
#############
cd ~/Desktop/$FOLDER

DIR3="75_obstacle_config_movable40_fixed120"
mkdir $DIR3
cd $DIR3

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR3 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100


#############
#TEST 4
#############
cd ~/Desktop/$FOLDER

DIR4="75_obstacle_config_movable40_fixed40"
mkdir $DIR4
cd $DIR4

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR4 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100

#############
#TEST 5
#############
cd ~/Desktop/$FOLDER

DIR5="75_obstacle_config_movable60_fixed60"
mkdir $DIR5
cd $DIR5

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR5 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100


#############
#TEST 6
#############
cd ~/Desktop/$FOLDER

DIR6="75_obstacle_config_movable120_fixed40"
mkdir $DIR6
cd $DIR6

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR6 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100




NUM_LINKS="three"
FOLDER="3_links_validate/"

#############
#TEST 1
#############

cd ~/Desktop/$FOLDER

DIR1="75_obstacle_config_movable20_fixed20"
mkdir $DIR1
cd $DIR1

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR1 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100


#############
#TEST 2
#############
cd ~/Desktop/$FOLDER

DIR2="75_obstacle_config_movable80_fixed80"
mkdir $DIR2
cd $DIR2

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR2 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100

#############
#TEST 3
#############
cd ~/Desktop/$FOLDER

DIR3="75_obstacle_config_movable40_fixed120"
mkdir $DIR3
cd $DIR3

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR3 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100


#############
#TEST 4
#############
cd ~/Desktop/$FOLDER

DIR4="75_obstacle_config_movable40_fixed40"
mkdir $DIR4
cd $DIR4

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR4 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100

#############
#TEST 5
#############
cd ~/Desktop/$FOLDER

DIR5="75_obstacle_config_movable60_fixed60"
mkdir $DIR5
cd $DIR5

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR5 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100


#############
#TEST 6
#############
cd ~/Desktop/$FOLDER

DIR6="75_obstacle_config_movable120_fixed40"
mkdir $DIR6
cd $DIR6

~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/scripts/run_simulation_multiple.sh ~/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/$DIR6 qs_slow_mpc use_skin single_reach multi_link_$NUM_LINKS 100

