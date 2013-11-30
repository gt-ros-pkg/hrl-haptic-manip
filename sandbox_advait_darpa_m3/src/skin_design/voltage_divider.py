

import numpy as np, math
import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')

import hrl_lib.matplotlib_util as mpu



# trying to see if it makes any difference if I choose R1 or R2 to be
# the variable resistor in the voltage divider.
#
# Answer: NO. Something I should have realized without having to make
# a plot.
def which_variable():
    r_static = 2e3
    r_min = 800.
    r_max = 5e3
    r_step = 10.
    r_var = np.arange(r_min, r_max, r_step)

    v_cc = 5.

    # case I - variable resistor is R2
    v1 = r_var / (r_static + r_var) * v_cc
    # case II - variable resistor is R1
    v2 = r_static / (r_static + r_var) * v_cc

    mpu.figure()
    pp.plot(r_var, v1, 'b', label='variable R2')
    pp.plot(r_var, v2, 'g', label='variable R1')
    pp.xlabel('Variable Resistance')
    pp.ylabel('Voltage')

    mpu.legend()
    pp.show()


# what is the effect of the pull up resistance. Can I choose a value
# that is good for our application?
def pull_up_resistor_value(rmax, rmin):
    n_r = 200
    n_pullups = 10
    adc_counts = 1024
    
    pullup_best = math.sqrt(rmax*rmin)
    pullup_max = 2 * pullup_best
    pullup_min = 0.5 * pullup_best
    pullup_step = (pullup_max - pullup_min) / n_pullups
    pullup_arr = np.arange(pullup_min, pullup_max, pullup_step)

    r_step = (rmax - rmin) / n_r
    r_var = np.arange(rmin, rmax, r_step)

    v_cc = 5.
    v_diff_list = []

    mpu.figure()
    for r_static in pullup_arr:
        v = r_var / (r_static + r_var) * v_cc
        pp.plot(r_var, v, mpu.random_color(), label='R1: %.1f'%r_static)
        v_diff_list.append(v[-1] - v[0])

    pp.xlabel('Variable Resistance')
    pp.ylabel('Voltage')

    mpu.legend()

    mpu.figure()
    pp.plot(pullup_arr, v_diff_list)
    pp.axvline(pullup_best, c='k', label='Analytically computed optimal value')
    pp.xlabel('Pull up resistance (ohms)')
    pp.ylabel('Difference in Voltage')
    mpu.legend()

    l1 = (r_static + rmin) / (r_static + rmax) * adc_counts
    l2 = rmin / rmax * (r_static + rmax) / (r_static + rmin) * adc_counts

    print 'ADC lost if piezo to GND:', l2
    print 'ADC lost if piezo to Vcc:', l1

    pp.show()



if __name__ == '__main__':
    #which_variable()

    # Velostat
    rmax = 1.4e3
    rmin = 100

    # Eeonyx LTT-SL-PA-MM-1-58B
    #rmax = 15e3
    #rmin = 1000

    # Eeonyx LR-SL-PA-MM-1-54
    #rmax = 15e3
    #rmin = 1500

    # Eeonyx LVY-SL-PA-10E6 RP-3-89-3
    #rmax = 1.5e6
    #rmin = 150e3

    # Eeonyx NW170-SL-PA
    #rmax = 1.5e3
    #rmin = 130

    pull_up_resistor_value(rmax, rmin)




