import numpy as np

import rospy

from hrl_msgs.msg import FloatArray

import force_torque.FTClient as ftc
import hrl_lib.util as ut


def adc_data_cb(msg):
    global adc_value
    adc_value = msg.data[0]

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--collect_data', '--cd', action='store_true',
                 dest='cd',
                 help='collect FT and ADC data and save to pkl')

    p.add_option('--pull_up', '-p', action='store', dest='pull_up',
                 type=float, help='pull up resistor value')

    p.add_option('--contact_area', '--ca', action='store', dest='ca',
                 type=float, help='contact area (m^2)')

    p.add_option('--viz_data', '--vd', action='store_true',
                 dest='vd',
                 help='visualize data collected with --cd')

    opt, args = p.parse_args()

    global adc_value
    adc_value = 0

    if opt.cd:
        if not opt.pull_up:
            raise RuntimeError('provide a value for the pull-up resistor')
        if not opt.ca:
            raise RuntimeError('provide a value for the contact area')

        rospy.Subscriber('adc_data', FloatArray, adc_data_cb)

        rospy.init_node('taxel_calib_data_collector')

        ft_client = ftc.FTClient('force_torque_ft1')
        ft_client.bias()

        rospy.loginfo('waiting to get adc_value')
        while adc_value == 0:
            rospy.sleep(0.1)

        adc_bias = adc_value

        raw_input('Hit ENTER to start recording data')

        duration = 20.
        t0 = rospy.get_time()
        t1 = t0 + duration

        ft_l = []
        adc_l = []
        while rospy.get_time() < t1:
            rospy.sleep(0.02)
            adc_l.append(adc_value)
            ft_l.append(ft_client.read()[2, 0])

        d = {}
        d['adc'] = adc_l
        d['ft'] = ft_l
        d['adc_bias'] = adc_bias
        d['pull_up'] = opt.pull_up
        d['contact_area'] = opt.ca

        ut.save_pickle(d, 'taxel_ft_calib_data.pkl')

    if opt.vd:
        import matplotlib.pyplot as pp
        import hrl_lib.matplotlib_util as mpu

        d = ut.load_pickle('taxel_ft_calib_data.pkl')
        ft_l = d['ft']
        adc_l = (d['adc_bias'] - np.array(d['adc'])).tolist()

        print np.max(ft_l)

        mpu.figure()
        pp.scatter(adc_l, ft_l, marker='x')
        pp.xlabel('ADC bias - ADC')
        pp.ylabel('FT_z')
        pp.show()
