
import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')

import hrl_lib.util as ut


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--dir', '-d', action='store',
                 dest='direc', type='string', default=None,
                 help='directory with result pkls from openrave')

    opt, args = p.parse_args()

    nm_l = ut.get_bash_command_output('ls %s/openrave*.pkl'%opt.direc)

    success_trials_list = []
    success_count = 0
    fail_count = 0
    for nm in nm_l:
        res_dict = ut.load_pickle(nm)
        if res_dict['result'] == 'Reached':
            success_count += 1
            success_trials_list.append(nm[:-4].split('/')[-1][9:])
        else:
            fail_count += 1

    print success_count, 'successful reaches'
    print fail_count, 'failed reaches'
    print success_count, fail_count, 'total trials'
    print 'Success rate:', success_count * 100. / (success_count + fail_count)

    d = {}
    d['successful_trials'] = success_trials_list
    d['success_count'] = success_count
    d['fail_count'] = fail_count
    ut.save_pickle(d, opt.direc+'/combined_results.pkl')


