#!/bin/bash -x

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed000_movable020_05N/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable020_fixed000/ --f_thresh=5 --t_impulse=0.7


python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed000_movable080_05N/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable080_fixed000/ --f_thresh=5 --t_impulse=0.7

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed000_movable020_25N/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable020_fixed000/ --f_thresh=25 --t_impulse=0.2


python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed000_movable080_25N/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable080_fixed000/ --f_thresh=25 --t_impulse=0.2


python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed010_movable010_05N/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable010_fixed010/ --f_thresh=5 --t_impulse=0.7


python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed040_movable040_05N/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable040_fixed040/ --f_thresh=5 --t_impulse=0.7

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed010_movable010_25N/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable010_fixed010/ --f_thresh=25 --t_impulse=0.2

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed040_movable040_25N/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable040_fixed040/ --f_thresh=25 --t_impulse=0.2



#this is to see effect of slowing down with stiffer joints
python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.01_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=0.01

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.10_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=0.10

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.33_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=0.33

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.50_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=0.50

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.66_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=0.66

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_1.00_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=1.00

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_2.00_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=2.00

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_3.00_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=3.00

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_4.00_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=4.00

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_5.00_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=5.00

python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_6.00_fixed_080_movable_000_05N_threshold_with_damping_new_impulse_model_t_impulse_0.4/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=5 --t_impulse=0.4 --imp_scale=6.00



# #for 25 N threshold
# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.01_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=0.01

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.10_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=0.10

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.33_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=0.33

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.50_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=0.50

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_0.66_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=0.66

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_1.00_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=1.00

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_2.00_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=2.00

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_3.00_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=3.00

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_4.00_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=4.00

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_5.00_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=5.00

# python /home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/run_multiple_trials.py --save_path=/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_6.00_fixed_080_movable_000_25N_threshold_with_damping_new_impulse_model/ --load_path=/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/ --f_thresh=25 --t_impulse=0.2 --imp_scale=6.00


# 0.01
# 0.10
# 0.33
# 0.5
# 0.66
# 1.00
# 2.0
# 3.0
# 4.0
# 5.0
# 6.0
