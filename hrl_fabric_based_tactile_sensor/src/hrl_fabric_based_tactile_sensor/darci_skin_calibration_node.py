#!/usr/bin/env python

import rospy

import hrl_meka_skin_sensor_darpa_m3.skin_patch_calibration as spc

from std_msgs.msg import Empty


class Fabric_Skin_Calibration(spc.SkinCalibration):
    def __init__(self):
        spc.SkinCalibration.__init__(self)

    # def raw_data_to_force(self, raw_data):
    #     # this might change depending on the pull-up value (e.g.
    #     # different pullup values on the PR2 and Cody)
    #     print "got into raw_data_to_force"
    #     try:
    #         print "got into trying"
    #         #d_biased = self.subtract_bias(raw_data, 0)
    #         #calib_data = self.adc_to_force_func(raw_data)

    #         biased = self.bias_mn - raw_data
    #         print "biased is :\n", biased
    #         calib_data = np.array([(0.17*math.exp(0.001378*x)) for x in biased])
    #         print "calib is :\n", calib_data
    #         print "got past adc_to_force_func"
    #         idxs = (np.where(calib_data < self.max_ignore_value))[0]
    #         calib_data[idxs] = 0.
    #         return calib_data
    #     except ValueError:
    #         rospy.logerr('raw_data.shape: '+str(raw_data.shape))
    #         rospy.signal_shutdown('Error in the fabric skin driver or calibration node')


def zero_sensor_cb(msg):
    fsc.compute_bias(rdc, 10)


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--slope', action='store',
                 dest='slope', type='float',
                 help='slope of calibration line')

    p.add_option('--max_ignore_value', '--miv', action='store',
                 dest='miv', type='float',
                 help='max force to ignore (return as zero)')

    opt, args = p.parse_args()

    rospy.init_node('fabric_skin_calibration_node')

    fsc = Fabric_Skin_Calibration()
    fsc.precompute_taxel_location_and_normal()
    fsc.calibration_slope = opt.slope
    fsc.max_ignore_value = opt.miv

    rdc = spc.RawDataClient('taxels/raw_data')
    fsc.compute_bias(rdc, 100)

    rospy.Subscriber('zero_sensor', Empty, zero_sensor_cb)

    while not rospy.is_shutdown():
        d = rdc.get_raw_data(True)
        fsc.publish_taxel_array(d, opt.slope)
