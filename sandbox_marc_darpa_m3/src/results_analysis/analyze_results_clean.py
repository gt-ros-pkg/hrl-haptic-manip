

import numpy as np, math
import sys, os

import roslib; roslib.load_manifest('darpa_m3')

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu
import matplotlib.pyplot as pl
import cPickle

flags = {}
flags['success_only'] = False
flags['all_forces'] = True
flags['median_force'] = True
flags['avg_force'] = True
flags['max_force'] = True


def compare(res_dict, c1, c2):
    c1_successes = set(res_dict[c1]['successes'])
    c2_successes = set(res_dict[c2]['successes'])

    print 'Both', c1, 'and', c2, 'successful:'
    print c1_successes.intersection(c2_successes)

    print ''
    print c1, 'successful, but not', c2
    print c1_successes.difference(c2_successes)

    print ''
    print c2, 'successful, but not', c1
    print c2_successes.difference(c1_successes)

def compare_all(res_dict, c_list, alpha_list, max_force_thresh):
    i = 0
    median_force = []
    med_max_force = []
    max_max_force = []
    avg_force = []
    avg_time_to_complete = []
    med_time_to_complete = []
    successes = []
    max_force_trials = []
    med_velocity = []
    length = len(res_dict[c_list[0]]['success'])

        # if flags['success_only'] == True:
        #     mag_contact_forces = [logger_res['mag_contact_forces'][r-1] for r in logger_dict['success_intersection']]
    for c in c_list:
        success_velocities = [res_dict[c]['avg_vel'][r-1] for r in res_dict["success_intersection"]]
        med_velocity.append(np.median(success_velocities))
        #med_velocity.append(np.median(res_dict[c]['avg_vel']))
        max_forces = []
        max_force_inds = []
        for i in xrange(len(res_dict[c]['max_force'])):
            if res_dict[c]['max_force'][i] > max_force_thresh:
                max_forces.append(res_dict[c]['max_force'][i])
                max_force_inds.append(i+1)
        if max_forces != []:
            max_force_trials.append([c, max_forces, max_force_inds])
        else:
            max_force_trials.append([c, 0, 0])
        #max_force_trials.append([c, np.where(res_dict[c]['max_force'] == np.max(res_dict[c]['max_force']))[0][0]+1, np.max(res_dict[c]['max_force'])])
        median_force.append(np.median(res_dict[c]['mag_contact_forces']))
        avg_force.append(np.mean(res_dict[c]['mag_contact_forces']))
        successes.append(np.sum(np.where(np.array(res_dict[c]['success']) == True, 1, 0)))
        avg_time_to_complete.append(np.mean(res_dict[c]['time']))
        med_time_to_complete.append(np.median(res_dict[c]['time']))
        med_max_force.append(np.median(res_dict[c]['max_force']))
        max_max_force.append(np.max(res_dict[c]['max_force']))

    print "max force trials are : \n"
    for text in max_force_trials:
        print text

    pl.figure()
    pl.plot(alpha_list, med_velocity)
    pl.xlabel('values for alpha')
    pl.ylabel('median of average velocities (m/s)')
    pl.title('Median of average velocities as a function \n of single variable change in compliance')

    pl.figure()
    pl.plot(alpha_list, median_force)
    pl.xlabel('values for alpha')
    pl.ylabel('median forces (N)')
    pl.title('Median Force as a function of single variable change in compliance')

    pl.figure()
    pl.plot(alpha_list, avg_force)
    pl.xlabel('values for alpha')
    pl.ylabel('avg forces (N)')
    pl.title('Average Force as a function of single variable change in compliance')

    pl.figure()
    pl.plot(alpha_list, max_max_force)
    pl.xlabel('values for alpha')
    pl.ylabel('max forces (N)')
    pl.title('Max of max Forces as a function of single variable change in compliance')

    pl.figure()
    pl.plot(alpha_list, med_max_force)
    pl.xlabel('values for alpha')
    pl.ylabel('max forces (N)')
    pl.title('Median of max Forces as a function of single variable change in compliance')

    pl.figure()
    pl.plot(alpha_list, avg_time_to_complete)
    pl.xlabel('values for alpha')
    pl.ylabel('avg time to complete (s)')
    pl.title('Average time to complete as a function of single variable change in compliance')

    pl.figure()
    pl.plot(alpha_list, med_time_to_complete) 
    pl.xlabel('values for alpha')
    pl.ylabel('median time to complete (s)')
    pl.title('Median time to complete as a function of single variable change in compliance')

    pl.figure()
    pl.plot(alpha_list, successes)
    pl.xlabel('values for alpha')
    pl.ylabel('total # of successes (out of '+str(length)+')')
    pl.title('Success as a function of single variable change in compliance')


def build_controller_result_dict(c_list, start, end):
    res_dict = {}
    for c in c_list:
        res_dict[c] = {'successes': [], 'failures': []}

    for i in range(start, end+1):
        folder_nm = str(i).zfill(4)

        for c in c_list:
            if 'openrave' in c:
                f = open(folder_nm+'/'+c+'.pkl', 'r')
                res = cPickle.load(f)
            else:
                f = open(folder_nm+'/'+c+'_controller.pkl', 'r')
                res = cPickle.load(f)

            if res is None:
                print 'Trial', i, 'for', c, 'was None'
                continue

            if res['result'] == 'Reached':
                res_dict[c]['successes'].append(i)
            else:
                res_dict[c]['failures'].append(i)
    res_dict['trials']=[start, end]
    #ut.save_pickle(res_dict, 'compiled_res_dict.pkl')
    return res_dict


def analyze_success_failure(c_list, res_dict):

#    trials_with_solution = len(res_dict['openrave_result_ignore_moveable']['successes'])
    print '--------------------'
    print 'Number of successes:'
    for c in c_list:
        print c+' - ', len(res_dict[c]['successes'])

    # if trials_with_solution != 0:
    #     print '--------------------'
    #     print 'Percentage success:'
    #     for c in c_list:
    #         print c+' - ', len(res_dict[c]['successes']) * 100. / trials_with_solution


def get_controller_log_results(folder_nm, c, res_dict, success_ind):
    print folder_nm+'/'+c+'_logger.pkl'
    file_logger = open(folder_nm+'/'+c+'_logger.pkl', 'r')
    file_controller = open(folder_nm+'/'+c+'_controller.pkl', 'r')
    file_setup = open(folder_nm+'/reach_problem_dict.pkl', 'r')
    
    data_setup = cPickle.load(file_setup)
    res_log = cPickle.load(file_logger)
    res_con = cPickle.load(file_controller)

    file_setup.close()
    file_logger.close()
    file_controller.close()

    if res_log is None or res_con is None:
        print 'Trial', i, 'for', c, 'was None'
        return res_dict, success_ind

    if 'high force' in res_con['result']:
        res_dict[c]['results'].append('high force')
    else:
        res_dict[c]['results'].append(res_con['result'])
    res_dict[c]['success'].append(res_con['result'] == 'Reached')
    res_dict[c]['time'].append(res_log['total_approx_time'])
    res_dict[c]['max_force'].append(res_log['max_force'])   #I'm not sure which of these two lines I'm using elsewhere ...
    res_dict[c]['max_forces'].append(res_log['max_force'])  #I'm not sure which of these two lines I'm using elsewhere ...
    res_dict[c]['avg_force'].append(res_log['avg_force'])
    forces = res_log['forces']
    median_force_buf = []
    max_force_time = 0
    max_force_diff = res_dict[c]['max_force'][-1]
    max_force_loc = []
    max_force_ls = []
    i = 0
    for time_step_forces in forces:
        for force in time_step_forces:
            res_dict[c]['mag_contact_forces'].append(np.linalg.norm(force))
            median_force_buf.append(np.linalg.norm(force))
            if res_dict[c]['max_force'][-1] - np.linalg.norm(force) < max_force_diff:
                max_force_diff = res_dict[c]['max_force'][-1] - np.linalg.norm(force)
                max_force_time = res_log['f_time'][i]
                max_force_loc = res_log['contact_points_x_y_z'][i]
                max_force_ls = time_step_forces  #this is called time step, but is the forces at that time step i believe
        i = i+1
    if median_force_buf == []:
        res_dict[c]['median_forces'].append(0.0)
    else:
        res_dict[c]['median_forces'].append(np.median(median_force_buf))

    try:
        res_dict[c]['max_force_time'].append(max_force_time)
        res_dict[c]['max_force_loc'].append(max_force_loc)
        res_dict[c]['max_force_ls'].append(max_force_ls)
        angle_time_ind = None
        time_min = max_force_time
        j = 0

        for time in res_log['dist_and_angles_time']:
            if abs(max_force_time-time) < time_min:
                time_min = max_force_time-time
                angle_time_ind = j
            j = j+1

        if angle_time_ind == None:
            res_dict[c]['max_force_arm_angles'].append(res_log['arm_angles'][-1])
        else:
            res_dict[c]['max_force_arm_angles'].append(res_log['arm_angles'][angle_time_ind])

    except KeyError:
        print "THE MAX FORCE TIME AND OTHER STATS IS NOT IN THIS FILE"


    res_dict[c]['time_to_complete_list'].append(res_log['total_approx_time'])

    dist = 0.0
    ####NEED TO FIX THIS TO BE arm independent
    start = res_log['ee_position'][0]
    #start =  arm.kinematics.FK(res_log['arm_angles'][0])[0] 
    ####NEED TO FIX THIS TO BE arm independent (need goal in one of pkl files
    end = data_setup['goal'] #arm.kinematics.FK(res_log['arm_angles'][-1])[0] 
    ####NEED TO FIX THIS TO BE arm independent (need goal in one of pkl files


    rms_error = 0
    i = 0
    for i in xrange(len(res_log['arm_angles'])-1):
        pos = arm.kinematics.FK(res_log['arm_angles'][i])[0]
        buf_term = np.abs((end[0]-start[0])*(start[1]-pos[1])-(start[0]-pos[0])*(end[1]-start[1]))/np.sqrt(np.power(end[0]-start[0], 2)+np.power(end[1]-start[1],2))
        rms_error = rms_error + np.power(buf_term,2)
        dist = dist + np.linalg.norm( arm.kinematics.FK(res_log['arm_angles'][i+1])[0] 
                                      - arm.kinematics.FK(res_log['arm_angles'][i])[0] )
    print "i is :", i
    rms_error = np.sqrt(rms_error/float(i))
    print "rms error is :", rms_error
    res_dict[c]['rms_error'].append(rms_error)

    res_dict[c]['avg_vel'].append(dist/res_log['total_approx_time'])
    success_ind = success_ind and (res_con['result'] == 'Reached')
    return res_dict, success_ind


def build_logger_result_dict(path, c_list, start, end, res_dict, trial_list = None):
    # res_dict = {}
    # for c in c_list:
    #     res_dict[c] = {'time': [], 'max_force': [], 'avg_force': [],
    #                    'success': [], 'mag_contact_forces': [], 
    #                    'time_to_complete_list': [], 'max_forces': [], 
    #                    'median_forces' : [], 'results' : [], 'max_force_trial': [],
    #                    'avg_vel': [] }
    #     res_dict["success_intersection"] = []
    if trial_list == None:
        for i in range(start, end+1):
            folder_nm = path+str(i).zfill(4)

            success_ind = True
            for c in c_list:
                res_dict, success_ind = get_controller_log_results(folder_nm, c, res_dict, success_ind)
            if success_ind == True:
                res_dict["success_intersection"].append(i)
        res_dict['trials']=[start, end]
    else:
        for i in trial_list:
            folder_nm = path+str(i).zfill(4)

            success_ind = True
            for c in c_list:
                res_dict, success_ind = get_controller_log_results(folder_nm, c, res_dict, success_ind)
            if success_ind == True:
                res_dict["success_intersection"].append(i)
        res_dict['trials']=trial_list
    #ut.save_pickle(res_dict, 'compiled_logger_dict.pkl')
    return res_dict

def set_axis_limits(dict_x_lim, dict_y_lim, cur_x_lim, cur_y_lim):
    if dict_x_lim[0] > cur_x_lim[0]:
        dict_x_lim[0] = cur_x_lim[0]
    if dict_x_lim[1] < cur_x_lim[1]:
        dict_x_lim[1] = cur_x_lim[1]
    if dict_y_lim[0] > cur_y_lim[0]:
        dict_y_lim[0] = cur_y_lim[0]
    if dict_y_lim[1] < cur_y_lim[1]:
        dict_y_lim[1] = cur_y_lim[1]

def plot_result_histograms(c_list, logger_dict):
    #data_tests_failed = np.array([0]*len(logger_dict[c_list[0]]['results']))
    data_failed_cases = np.array([])
    for c in c_list:
        logger_res = logger_dict[c]
        pl.figure()
        trans = {'Reached':0, 'timed out':1, 'high force':2}
        data_results = [trans[logger_res['results'][i]] for i in xrange(len(logger_res['results']))]
        failed_ind = np.hstack(( np.where(np.array(data_results)==1), 
                                 np.where(np.array(data_results)==2) ))   ###########check thissss!!!!!!!!!!
        #print "failed_ind \n", failed_ind+1
        #print "data_failed_cases ", data_failed_cases
        data_failed_cases = np.hstack( (data_failed_cases, failed_ind.flatten()+1) )
        #data_tests_failed[failed_ind] = data_tests_failed[failed_ind] + 1
        pl.hist(data_results)
    if data_failed_cases != np.array([]):
        pl.figure()
        pl.hist(data_failed_cases, len(logger_dict[c_list[0]]['results']))
        pl.title('histogram of failed cases vs test case #')
    else:
        pl.figure()
        pl.title('there were no failed cases')

def get_high_force_cases(c_list, logger_dict, high_force):
    high_force_dict = {}
    for c in c_list:
        logger_res = logger_dict[c]
        high_force_list = []
        high_force_list = (np.where(np.array(logger_res['max_force'])>high_force)[0]+1).tolist()
        print "for controller: ", c
        print "list where forces exceeded "+str(high_force)+": "
        print high_force_list
        high_force_dict[c] = high_force_list
    return high_force_dict

def plot_force_histograms(c_list, logger_dict):
    force_plots = {'mag_contact_limits_x':[0, 0], 'mag_contact_limits_y':[0, 0],
                   'max_force_limits_x':[0, 0], 'max_force_limits_y':[0, 0],
                   'med_force_limits_x':[0, 0], 'med_force_limits_y':[0, 0], 
                   'avg_force_limits_x':[0, 0], 'avg_force_limits_y':[0, 0], 
                   'mag_contact': [], 'max_force': [], 'median_force': [], 'avg_force': []}

    for c in c_list:
        logger_res = logger_dict[c]

        if flags['success_only'] == True:
            mag_contact_forces = [logger_res['mag_contact_forces'][r-1] for r in logger_dict['success_intersection']]
            max_forces = [logger_res['max_forces'][r-1] for r in logger_dict['success_intersection']]
            median_forces = [logger_res['median_forces'][r-1] for r in logger_dict['success_intersection']]
            avg_forces = [logger_res['avg_force'][r-1] for r in logger_dict['success_intersection']]
            title_mag_contact_forces = 'contact forces for only successful trials for \n'+c
            title_max_forces = 'max force per trial for only successful trials for \n'+c
            title_median_forces = 'median force per trial for only successful trials for \n'+c
            title_avg_forces = 'avg force per trial for only successful trials for \n'+c
        else:
            mag_contact_forces = logger_res['mag_contact_forces']
            max_forces = logger_res['max_forces']
            median_forces = logger_res['median_forces']
            avg_forces = logger_res['avg_force']
            title_mag_contact_forces = 'contact forces for all trials for \n'+c
            title_max_forces = 'max force per trial for all trials for \n'+c
            title_median_forces = 'median force per trial for all trials for \n'+c
            title_avg_forces = 'avg force per trial for all trials for \n'+c

        if flags['all_forces'] == True:
            force_plots['mag_contact'].append(pl.figure())
            pl.hist(mag_contact_forces, bins=30, normed=True)
            set_axis_limits(force_plots['mag_contact_limits_x'], 
                            force_plots['mag_contact_limits_y'], 
                            pl.xlim(), pl.ylim())
            pl.title(title_mag_contact_forces)

        if flags['max_force'] == True:
            force_plots['max_force'].append(pl.figure())
            pl.hist(max_forces, bins=30, normed=True)
            set_axis_limits(force_plots['max_force_limits_x'], 
                            force_plots['max_force_limits_y'], 
                            pl.xlim(), pl.ylim())
            pl.title(title_max_forces)

        if flags['median_force'] == True:
            force_plots['median_force'].append(pl.figure())
            pl.hist(median_forces, bins=30, normed=True)
            set_axis_limits(force_plots['med_force_limits_x'], 
                            force_plots['med_force_limits_y'], 
                            pl.xlim(), pl.ylim())
            pl.title(title_median_forces)

        if flags['avg_force'] == True:
            force_plots['avg_force'].append(pl.figure())
            pl.hist(avg_forces, bins=30, normed=True)
            set_axis_limits(force_plots['avg_force_limits_x'], 
                            force_plots['avg_force_limits_y'], 
                            pl.xlim(), pl.ylim())
            pl.title(title_avg_forces)

    for i in xrange(len(c_list)):
        if flags['all_forces'] == True:
            a = force_plots['mag_contact'][i].gca()
            a.set_xlim(force_plots['mag_contact_limits_x'])
            a.set_ylim(force_plots['mag_contact_limits_y'])
        if flags['max_force'] == True:
            b = force_plots['max_force'][i].gca()              
            b.set_xlim(force_plots['max_force_limits_x'])              
            b.set_ylim(force_plots['max_force_limits_y'])
        if flags['median_force'] == True:
            c = force_plots['median_force'][i].gca()
            c.set_xlim(force_plots['med_force_limits_x'])
            c.set_ylim(force_plots['med_force_limits_y'])
        if flags['avg_force'] == True:
            d = force_plots['avg_force'][i].gca()
            d.set_xlim(force_plots['avg_force_limits_x'])
            d.set_ylim(force_plots['avg_force_limits_y'])

def analyze_time_to_completion(c_list, logger_dict, controller_dict):
    for c in c_list:
        logger_res = logger_dict[c]
        controller_res = controller_dict[c]
        pl.figure()
        time_array = np.array(logger_res['time_to_complete_list'])
        if flags['success_only'] == True:
            time_to_complete = [logger_res['time_to_complete_list'][r-1] for r in logger_dict['success_intersection']]
            title = 'time to completion for intersection of success for \n'+c
            success_ind = np.array(logger_dict['success_intersection']) - 1
            print "for intersection of successful trials:"
        else:
            time_to_complete = logger_res['time_to_complete_list']
            title = 'time to completion for successful trials for \n'+c
            success_ind = np.array(controller_res['successes']) - 1
            print "for all successful trials:"
        print "\t max time to complete for ", c, "is : ", np.max(time_array[success_ind])
        print "\t min time to complete for ", c, "is : ", np.min(time_array[success_ind])
        print "\t avg time to complete for ", c, "is : ", np.mean(time_array[success_ind])
        pl.hist(time_to_complete, bins=30)
        pl.title(title)

def analyze_forces(c_list, start, end):
    res_dict = {}
    for c in c_list:
        res_dict[c] = {'time': [], 'max_force': [], 'avg_force': [], 
                       'success': [], 'mag_contact_forces': [], 
                       'time_to_complete_list': [], 'max_forces': [], 
                       'median_forces' : [], 'results' : [], 'max_force_trial': [],
                       'avg_vel': [] , 'max_force_time':[]}
    res_dict["success_intersection"] = []

    logger_dict = build_logger_result_dict('./', c_list, start, end, res_dict)

    print '-------------------------'
    print 'force statistics:'
    for c in c_list:
        logger_res = logger_dict[c]
        succ_trials = np.where(logger_res['success'])
        max_force_arr = np.array(logger_res['max_force'])
        time_arr = np.array(logger_res['time'])
        print 'Mean max force for %s is %.2f'%(c, np.mean(max_force_arr))
        print 'Mean completion time for successful trials %s is %.2f'%(c, np.mean(time_arr[succ_trials]))
        print 'Mean max force for successful trials for %s is %.2f'%(c, np.mean(max_force_arr[succ_trials]))


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--start', action='store', dest='start',type='int',
                 default=None, help='first trial number')
    p.add_option('--end', action='store', dest='end',type='int',
                 default=None, help='last trial number')
    p.add_option('--config', action='store', dest='config',type='str',
                 default="analyze_config_file_default", help='name of the configuration file to use (defaults to analyze_config_file_default)')
    p.add_option('--use_common', action='store_true', dest='use_common',
                 default=False, help='use common pickle file that was generated from all results and previous controller list (be careful here)')
    p.add_option('--include_rave', action='store_true', dest='include_rave',
                 default=False, help='include openrave in list of controllers to analyze')
    p.add_option('--plot', action='store_true', dest='plot',
                 default=False, help='plot histograms of results')

    opt, args = p.parse_args()

    if opt.start == None or opt.end == None:
        print 'Please specify both start and end trial.'
        print 'Exiting ...'
        sys.exit()

    if opt.use_common == True:
        print 'BE AWARE, you are attempting to use a pkl file that was'
        print ' previously compiled from old results, is this correct?' 
        inp = raw_input('type y for yes \n')
        if inp != 'y':
            exit

    load_config = "from " + opt.config + " import *"
    # DANGER this could execute arbitrary code
    exec load_config

    if opt.plot == True:
        flags['no_plots'] = False
    else:
        flags['no_plots'] = True

    batch = None                
    #batch = [ 'F020_M050']
    #batch = [ 'F020_M050', 'F020_M100', 'F020_M200', 'F040_M050', 'F040_M100', 'F040_M200', 'F060_M050', 'F060_M100', 'F060_M200'] 

###############################should make an option to do this#####################
    if opt.use_common==True:
        logger_dict = ut.load_pickle('compiled_logger_dict.pkl')
        res_dict = ut.load_pickle('compiled_res_dict.pkl')
        if logger_dict == None or res_dict == None:
              res_dict = {}
              for c in controllers_list:
                  res_dict[c] = {'time': [], 'max_force': [], 'avg_force': [],
                                 'success': [], 'mag_contact_forces': [], 
                                 'time_to_complete_list': [], 'max_forces': [], 
                                 'median_forces' : [], 'results' : [], 'max_force_trial': [],
                                 'avg_vel': [], 'max_force_time': [], 'max_force_loc': [],
                                 'max_force_ls': [], 'max_force_arm_angles': []}
              res_dict["success_intersection"] = []
              logger_dict = build_logger_result_dict('./', controllers_list, opt.start, opt.end, res_dict)
              res_dict = build_controller_result_dict(controllers_list, opt.start, opt.end)
    elif batch != None:
        high_forces_dict = {}
        for path in batch:
            res_dict = {}
            for c in controllers_list:
                res_dict[c] = {'time': [], 'max_force': [], 'avg_force': [], 
                               'success': [], 'mag_contact_forces': [], 
                               'time_to_complete_list': [], 'max_forces': [], 
                               'median_forces' : [], 'results' : [], 'max_force_trial': [],
                               'avg_vel': [], 'max_force_time': [], 'max_force_loc': [],
                               'max_force_ls': [], 'max_force_arm_angles': []}
                res_dict["success_intersection"] = []
            logger_dict = build_logger_result_dict(path+'/', controllers_list, opt.start, opt.end, res_dict)
            high_forces_dict[path] = get_high_force_cases(controllers_list, logger_dict, 40)
            #res_dict = build_controller_result_dict(controllers_list, opt.start, opt.end)
        print high_forces_dict
        ut.save_pickle(high_forces_dict, 'trials_with_forces_above_40N_dict.pkl')
    else:
        res_dict = {}
        for c in controllers_list:
            res_dict[c] = {'time': [], 'max_force': [], 'avg_force': [],
                           'success': [], 'mag_contact_forces': [], 
                           'time_to_complete_list': [], 'max_forces': [], 
                           'median_forces' : [], 'results' : [], 'max_force_trial': [],
                           'avg_vel': [], 'max_force_time': [], 'max_force_loc': [],
                           'max_force_ls': [], 'max_force_arm_angles': [], 'rms_error':[],
                           'results':[] }
        res_dict["success_intersection"] = []
        logger_dict = build_logger_result_dict('./', controllers_list, opt.start, opt.end, res_dict)
        if flags['no_plots'] == False:
            logger_dict = build_logger_result_dict('./', controllers_list, opt.start, opt.end, res_dict)
            get_high_force_cases(controllers_list, logger_dict, 40)
        res_dict = build_controller_result_dict(controllers_list, opt.start, opt.end)

    # if flags['no_plots'] == False:
    #     plot_force_histograms(controllers_list, logger_dict)
    #     analyze_time_to_completion(controllers_list, logger_dict, res_dict)
    if opt.include_rave == True:
        #c_list = controllers_list + ['openrave_result_all', 'openrave_result_ignore_moveable']
        c_list = controllers_list + ['openrave_result_ignore_moveable']
        res_dict = build_controller_result_dict(c_list, opt.start, opt.end)        
    else:
        c_list = controllers_list

    analyze_success_failure(c_list, res_dict)

    # if flags['no_plots'] == False:
    #     pl.show()
    #compare(res_dict, controllers_list[0], controllers_list[1])
###################################################################################

    if False:
        res_dict = {}
        for c in controllers_list:
            res_dict[c] = {'time': [], 'max_force': [], 'avg_force': [],
                           'success': [], 'mag_contact_forces': [], 
                           'time_to_complete_list': [], 'max_forces': [], 
                           'median_forces' : [], 'results' : [], 'max_force_trial': [],
                           'avg_vel': [] }
        res_dict["success_intersection"] = []
        logger_dict = build_logger_result_dict('./', controllers_list, opt.start, opt.end, res_dict)
        res_dict = build_controller_result_dict(controllers_list, opt.start, opt.end)

        try:
            alpha_list
        except NameError:
            alpha_list = None

        if alpha_list != None:
            compare_all(logger_dict, controllers_list, alpha_list, 100)

        controllers_list2 = ['stiffness_alpha_'+str(1).zfill(3)+'_var_damping_qp_20110913_5N_max_100']
        [controllers_list2.append('stiffness_alpha_'+str((i+1)*100).zfill(3)+'_var_damping_qp_20110913_5N_max_100') for i in xrange(6)]

    #plot_result_histograms(controllers_list2, logger_dict)

    #pl.show()
    #compare(res_dict, 'stiffness_alpha_'+str(10).zfill(3)+'_var_damping_qp_20110913_5N_max_100', 'stiffness_alpha_'+str(400).zfill(3)+'_var_damping_qp_20110913_5N_max_100')
    #compare(res_dict, controllers_list[0], controllers_list[1])

#    analyze_forces(controllers_list, opt.start, opt.end)
#    analyze_time(controllers_list, opt.start, opt.end)
#    analyze_





##############code was in building logger function#################
            # print folder_nm+'/'+c+'_logger.pkl'
            # file_logger = open(folder_nm+'/'+c+'_logger.pkl', 'r')
            # file_controller = open(folder_nm+'/'+c+'_controller.pkl', 'r')
            # res_log = cPickle.load(file_logger)
            # res_con = cPickle.load(file_controller)

            # if res_log is None or res_con is None:
            #     print 'Trial', i, 'for', c, 'was None'
            #     continue

            # if 'high force' in res_con['result']:
            #     res_dict[c]['results'].append('high force')
            # else:
            #     res_dict[c]['results'].append(res_con['result'])
            # res_dict[c]['success'].append(res_con['result'] == 'Reached')
            # res_dict[c]['time'].append(res_log['total_approx_time'])
            # res_dict[c]['max_force'].append(res_log['max_force'])
            # res_dict[c]['avg_force'].append(res_log['avg_force'])
            # forces = res_log['forces']
            # median_force_buf = []
            # for time_step in forces:
            #     for force in time_step:
            #         res_dict[c]['mag_contact_forces'].append(np.linalg.norm(force))
            #         median_force_buf.append(np.linalg.norm(force))
            # if median_force_buf == []:
            #     res_dict[c]['median_forces'].append(0.0)
            # else:
            #     res_dict[c]['median_forces'].append(np.median(median_force_buf))
            # res_dict[c]['time_to_complete_list'].append(res_log['total_approx_time'])
            # dist = 0.0
            # for i in xrange(len(res_log['arm_angles'])-1):
            #     dist = dist + np.linalg.norm( arm.kinematics.FK(res_log['arm_angles'][i+1])[0] 
            #                                   - arm.kinematics.FK(res_log['arm_angles'][i])[0] )
            # res_dict[c]['avg_vel'].append(dist/res_log['total_approx_time'])
            # res_dict[c]['max_forces'].append(res_log['max_force'])
            # success_ind = success_ind and (res_con['result'] == 'Reached')
