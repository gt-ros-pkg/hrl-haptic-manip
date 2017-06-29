#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as pp

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu


def min_max_force_min_pressure_to_trigger_pressure_threshold(th, pressure_l, force_l, adc_l):
    p_arr = np.array(pressure_l)
    f_arr = np.array(force_l)
    adc_arr = np.array(adc_l)
    idxs = np.where(p_arr > th)[0]
    if len(idxs) == 0:
        return None, None, None, None
    adc_thresh = np.min(adc_arr[idxs])
    if adc_thresh == 0.:
        return None, None, None, None
    min_force = np.min(f_arr[np.where(adc_arr > adc_thresh)[0]])
    max_force = np.max(f_arr[np.where(adc_arr < adc_thresh)[0]])
    min_pressure = np.min(p_arr[np.where(adc_arr > adc_thresh)[0]])
    return min_force, max_force, min_pressure, adc_thresh


def min_force_to_trigger_threshold(threshold, force_l, adc_l):
    f_arr = np.array(force_l)
    adc_arr = np.array(adc_l)
    idxs = np.where(f_arr > threshold)[0]
    if len(idxs) == 0:
        return None, None
    adc_thresh = np.min(adc_arr[idxs])
    min_force = np.min(f_arr[np.where(adc_arr > adc_thresh)[0]])
    return min_force, adc_thresh


def uncertainty_in_force(force_l, adc_l, adc_bin_size):
    min_adc = min(adc_l)
    max_adc = max(adc_l)
    adc_val = min_adc + adc_bin_size / 2.

    f_arr = np.array(force_l)
    adc_arr = np.array(adc_l)
    adc_bin_l = []
    max_f_l = []
    min_f_l = []
    while adc_val < max_adc:
        idxs = np.where(np.all(np.row_stack((adc_arr < (adc_val+adc_bin_size/2.),
                                             adc_arr >= (adc_val-adc_bin_size/2))), 0))[0]
        if len(idxs) != 0:
            adc_bin_l.append(adc_val)
            max_f_l.append(np.max(f_arr[idxs]))
            min_f_l.append(np.min(f_arr[idxs]))
        else:
            print 'boo'
        adc_val += adc_bin_size

    return adc_bin_l, max_f_l, min_f_l


def plot_uncertainty_in_force(d, color):
    ft_l = d['ft']
    adc_l = (d['adc_bias'] - np.array(d['adc'])).tolist()
    adc_bin_l, max_f_l, min_f_l = uncertainty_in_force(ft_l, adc_l, 20)

    pp.plot(adc_bin_l, np.array(max_f_l) - np.array(min_f_l), color=color,
            label='.'.join('/'.join(nm.split('/')[1:]).split('.')[0:-1]))

    pp.xlabel('ADC bias - ADC')
    pp.ylabel('max - min of measured FT_z')
    pp.legend()


def force_vs_adc_2(d, color):
    ft_l = d['ft']
    adc_l = (d['adc_bias'] - np.array(d['adc'])).tolist()
    # adc_l = d['adc']

    f_prev = 0.
    temp_ft_l = []
    temp_adc_l = []
    increasing = True
    for i in range(len(ft_l)):
        f = ft_l[i]
        a = adc_l[i]
        if f > f_prev:
            if increasing:
                temp_ft_l.append(f)
                temp_adc_l.append(a)
            else:
                pp.plot(temp_adc_l, temp_ft_l, color='r', alpha=0.5)
                temp_ft_l = [f]
                temp_adc_l = [a]
            increasing = True
        else:
            if increasing:
                pp.plot(temp_adc_l, temp_ft_l, color='b', alpha=0.5)
                temp_ft_l = [f]
                temp_adc_l = [a]
            else:
                temp_ft_l.append(f)
                temp_adc_l.append(a)
            increasing = False
        f_prev = f

    pp.xlabel('ADC bias - ADC')
    pp.ylabel('FT_z')
    pp.legend()


def force_vs_adc(d, color):
    ft_l = d['ft']
    adc_l = (d['adc_bias'] - np.array(d['adc'])).tolist()
    # adc_l = d['adc']
    # pp.scatter(adc_l, ft_l, marker='x', color=color,
    # label='.'.join('/'.join(nm.split('/')[1:]).split('.')[0:-1]))

    pp.scatter(adc_l, ft_l, marker='x', color=color,
               label=nm.split('/')[-1].split('.')[0])

    pp.xlabel('ADC bias - ADC')
    pp.ylabel('FT_z')
    pp.legend()
    pp.grid('on')


def pressure_vs_adc(d, color):
    ft_l = d['ft']
    adc_l = (d['adc_bias'] - np.array(d['adc'])).tolist()
    # adc_l = d['adc']
    area = d['contact_area']
    press_l = [f/area * 0.001 for f in ft_l]
# pp.scatter(adc_l, press_l, marker='x', color=color,
#               label='.'.join('/'.join(nm.split('/')[1:]).split('.')[0:-1]))
    pp.scatter(adc_l, press_l, marker='x', color=color,
               label=nm.split('/')[-1].split('.')[0])

    pp.xlabel('ADC bias - ADC')
    pp.ylabel('Pressure (kPa)')
    pp.legend()
    pp.grid('on')


def resistance_from_adc(adc, adc_max, r1, r2):
    if adc_max == adc:
        return 5000.
    return (r1 * adc) / (adc_max - adc) - r2


def force_vs_resistance(d, color):
    ft_l = d['ft']
    adc_max = d['adc_bias'] * 1.
    r1 = d['pull_up']
    r2 = 40.  # conductive thread and connection between taxel and thread
    rv_l = [resistance_from_adc(adc, adc_max, r1, r2) for adc in d['adc']]

    pp.scatter(rv_l, ft_l, marker='x', color=color,
               label=nm.split('/')[-1].split('.')[0])

    pp.xlabel('variable resistance')
    pp.ylabel('FT_z')
    pp.xlim(0, 4000.)
    pp.legend()


def pressure_vs_resistivity(d, color):
    ft_l = d['ft']
    area = d['contact_area']
    adc_max = d['adc_bias'] * 1.
    r1 = d['pull_up']
    r2 = 0.  # conductive thread and connection between taxel and thread
    rv_l = [resistance_from_adc(adc, adc_max, r1, r2) for adc in d['adc']]

    press_l = [f/(area*1000.) for f in ft_l]
    rho_l = [r*area for r in rv_l]

    pp.scatter(press_l, rho_l, marker='x', color=color,
               label=nm.split('/')[-1].split('.')[0])

    pp.ylabel('resistivity (assuming length is constant)')
    pp.xlabel('pressure')
    pp.legend()


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--direc', '-d', action='store',
                 dest='direc', type='string',
                 help='directory with all the pkls (each pkl gets separate color)')

    opt, args = p.parse_args()

    if not opt.direc:
        print 'Please specify a directory'
        sys.exit()

    nm_l = ut.get_bash_command_output('find '+opt.direc+' -name "*.pkl"')

    color_list = ['r', 'g', 'b']
    for nm in nm_l:
        c = mpu.random_color()
        color_list.append(c)

    if True:
        mpu.figure()
        for nm, c in zip(nm_l, color_list):
            d = ut.load_pickle(nm)
            # d['adc_bias']=1023 # For stretching experiments with Sarvagya
            force_vs_adc(d, c)
            # force_vs_adc_2(d, c)
            pp.xlim((0, 1000))
            pp.ylim((-10, 80))

            print 'ADC bias: %d' % d['adc_bias']

#        mpu.figure()
#        for nm, c in zip(nm_l, color_list):
#            d = ut.load_pickle(nm)
#            plot_uncertainty_in_force(d, c)

    if False:
        print '====================================================================='
        print 'Min force that will trigger the safety threshold (known contact area)'
        print '====================================================================='

        force_threshold_l = [10., 15., 20., 25., 30.]
        fig1 = mpu.figure()
        pp.xlabel('Force Threshold')
        pp.ylabel('Min Triggered Force')
        pp.title('Known Contact Area')
        fig2 = mpu.figure()
        pp.xlabel('Force Threshold')
        pp.ylabel('Min Triggered ADC value')
        pp.title('Known Contact Area')
        for nm, c in zip(nm_l, color_list):
            label = '.'.join('/'.join(nm.split('/')[1:]).split('.')[0:-1])
            trig_f_l = []
            trig_adc_l = []
            thresh_l = []
            for force_threshold in force_threshold_l:
                d = ut.load_pickle(nm)
                # d['adc_bias']=1023 #For stretching experiments with Sarvagya
                ft_l = d['ft']
                adc_l = (d['adc_bias'] - np.array(d['adc'])).tolist()
                f, a = min_force_to_trigger_threshold(force_threshold, ft_l, adc_l)
                if f is None:
                    continue
                trig_f_l.append(f)
                trig_adc_l.append(a)
                thresh_l.append(force_threshold)
            pp.figure(fig1.number)
            pp.plot(thresh_l, trig_f_l, label=label, color=c, marker='o', ms=7)
            pp.figure(fig2.number)
            pp.plot(thresh_l, trig_adc_l, label=label, color=c, marker='o', ms=7)
        pp.figure(fig1.number)
        pp.legend()
        pp.figure(fig2.number)
        pp.legend()

    # combine a bunch of pkl
    if False:
        d = {}
        d['ft_l'] = []
        d['pull_up_l'] = []
        d['adc_l'] = []
        d['adc_bias_l'] = []
        d['contact_area_l'] = []
        for nm in nm_l:
            d_t = ut.load_pickle(nm)
            d['ft_l'].append(d_t['ft'])
            d['pull_up_l'].append(d_t['pull_up'])
            d['adc_l'].append(d_t['adc'])
            d['adc_bias_l'].append(d_t['adc_bias'])
            # d['adc_bias_l'].append(1023) # For stretching experiments with Sarvagya
            d['contact_area_l'].append(d_t['contact_area'])

        ut.save_pickle(d, 'combined.pkl')

    if False:
        print '======================================================================='
        print 'Min force that will trigger the safety threshold (unknown contact area)'
        print '======================================================================='

        force_threshold_l = [10., 15., 20., 25., 30.]
        fig1 = mpu.figure()
        pp.xlabel('Force Threshold')
        pp.ylabel('Min Triggered Force')
        pp.title('Unknown Contact Area')
        fig2 = mpu.figure()
        pp.xlabel('Force Threshold')
        pp.ylabel('Min Triggered ADC value')
        pp.title('Unknown Contact Area')

        ft_l = []
        adc_l = []
        for nm, c in zip(nm_l, color_list):
            label = '.'.join(nm.split('/')[-1].split('.')[0:-1])
            trig_f_l = []
            trig_adc_l = []
            thresh_l = []
            for force_threshold in force_threshold_l:
                d = ut.load_pickle(nm)
                # d['adc_bias']=1023 #For stretching experiments with Sarvagya
                ft_l = []
                adc_l = []
                for i in range(len(d['adc_bias_l'])):
                    ft_l += d['ft_l'][i]
                    bias = d['adc_bias_l'][i]
                    adc_l += (bias - np.array(d['adc_l'][i])).tolist()
                f, a = min_force_to_trigger_threshold(force_threshold, ft_l, adc_l)
                if f is None:
                    continue
                trig_f_l.append(f)
                trig_adc_l.append(a)
                thresh_l.append(force_threshold)
            pp.figure(fig1.number)
            pp.plot(thresh_l, trig_f_l, label=label, color=c, marker='o', ms=7)
            pp.figure(fig2.number)
            pp.plot(thresh_l, trig_adc_l, label=label, color=c, marker='o', ms=7)
        pp.figure(fig1.number)
        pp.legend()
        pp.figure(fig2.number)
        pp.legend()

    if False:
        print '======================================================================='
        print 'Forces that will trigger the safety threshold on pressure (unknown contact area)'
        print '======================================================================='

        pressure_threshold_l = [50000., 100000, 150000., 200000.,
                                250000., 300000., 350000., 400000.,
                                450000., 500000.]
        fig1 = mpu.figure()
        pp.xlabel('Force Threshold')
        pp.ylabel('Min Triggered Force and max permitted force')
        pp.title('Unknown Contact Area')
        fig2 = mpu.figure()
        pp.xlabel('Pressure Threshold')
        pp.ylabel('Min Triggered ADC value')
        pp.title('Unknown Contact Area')
        fig3 = mpu.figure()
        pp.xlabel('Pressure Threshold')
        pp.ylabel('Min Triggered Pressure')
        pp.title('Unknown Contact Area')

        ft_l = []
        adc_l = []
        for nm, c in zip(nm_l, color_list):
            label = '.'.join(nm.split('/')[-1].split('.')[0:-1])
            trig_f_min_l = []
            trig_f_max_l = []
            trig_p_l = []
            trig_adc_l = []
            thresh_l = []
            for th in pressure_threshold_l:
                d = ut.load_pickle(nm)
                # d['adc_bias']=1023 #For stretching experiments with Sarvagya
                ft_l = []
                pressure_l = []
                adc_l = []
                for i in range(len(d['adc_bias_l'])):
                    ca = d['contact_area_l'][i]
                    ft_l += d['ft_l'][i]
                    pressure_l += (np.array(d['ft_l'][i])/ca).tolist()

                    bias = d['adc_bias_l'][i]
                    adc_l += (bias - np.array(d['adc_l'][i])).tolist()

                f_min, f_max, p_min, a = min_max_force_min_pressure_to_trigger_pressure_threshold(th, pressure_l, ft_l, adc_l)
                if f_min is None:
                    continue

                trig_f_min_l.append(f_min)
                trig_f_max_l.append(f_max)
                trig_adc_l.append(a)
                trig_p_l.append(p_min)
                thresh_l.append(th)
            pp.figure(fig1.number)
            pp.plot(thresh_l, trig_f_min_l, label=label, color=c, marker='o', ms=7)
            pp.plot(thresh_l, trig_f_max_l, label=label, color=c, marker='o', ms=7)
            pp.figure(fig2.number)
            pp.plot(thresh_l, trig_adc_l, label=label, color=c, marker='o', ms=7)
            pp.figure(fig3.number)
            pp.plot(thresh_l, trig_p_l, label=label, color=c, marker='o', ms=7)
        pp.figure(fig1.number)
        pp.legend()
        pp.figure(fig2.number)
        pp.legend()
        pp.figure(fig3.number)
        pp.legend()

#    mpu.figure()
#    for nm, c in zip(nm_l, color_list):
#        d = ut.load_pickle(nm)
#        force_vs_resistance(d, c)
#
#    mpu.figure()
#    for nm, c in zip(nm_l, color_list):
#        d = ut.load_pickle(nm)
#        pressure_vs_resistivity(d, c)

    if False:
        mpu.figure()
        for nm, c in zip(nm_l, color_list):
            d = ut.load_pickle(nm)
            # d['adc_bias']=1023 #For stretching experiments with Sarvagya
            pressure_vs_adc(d, c)

    pp.show()
