

import matplotlib.pyplot as pp
import numpy as np, math

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')


def one_over_x(x):
    return 1/x

def one_over_x_square(x):
    return 1./(x**2.5)

def compute_rho(p):
    return 300. * one_over_x_square(p/1000.)


def compute_sensor_output(rho, r1, a):
    l = 1.
    rv = rho * l / a
    adc = rv/(r1+rv) * 1023
    return adc


if __name__ == '__main__':

    if False:
        # testing the rho function
        p_arr = np.arange(10, 200, 2.) * 1000.
        #rho_list = map(one_over_x, p_arr)
        #rho_list = map(one_over_x_square, p_arr)
        rho_list = map(compute_rho, p_arr)
        pp.plot(p_arr / 1000., rho_list)
        pp.xlabel('pressure (kPa)')
        pp.ylabel('resistivity')
        pp.show()
    

    if True:
        f_arr = np.arange(1, 50, 0.1)
        a_list = [0.01**2, 0.02**2, 0.04**2]
        color_list = ['c', 'g', 'r']

        r1 = 470.

        pp.figure()
        for i in range(3):
            a = a_list[i]
            c = color_list[i]
            p_arr = f_arr/a
            rho_list = map(compute_rho, p_arr)
            adc_l = [compute_sensor_output(rho, r1, a) for rho in rho_list]
            lab = '%d cm'%(int(math.sqrt(a)*100))
            pp.scatter(adc_l, (p_arr/1000.).tolist(), marker='x', s=4, color=c,
                       label=lab)

        pp.xlabel('Tactile sensor output')
        pp.ylabel('Pressure (kPa)')
        pp.legend()

        pp.figure()
        for i in range(3):
            a = a_list[i]
            c = color_list[i]
            p_arr = f_arr/a
            rho_list = map(compute_rho, p_arr)
            adc_l = [compute_sensor_output(rho, r1, a) for rho in rho_list]
            lab = '%d cm'%(int(math.sqrt(a)*100))
            pp.scatter(adc_l, f_arr, marker='x', s=4, color=c,
                       label=lab)

        pp.xlabel('Tactile sensor output')
        pp.ylabel('Force (N)')
        pp.legend()

        pp.show()










