
import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('hrl_meka_skin_sensor_darpa_m3')
import rospy

import hrl_meka_skin_sensor_darpa_m3.skin_patch_calibration as spc

from m3skin_ros.srv import None_TransformArray
from m3skin_ros.srv import None_String
from m3skin_ros.msg import TaxelArray

class Fabric_Skin_Calibration(spc.SkinCalibration):
    def __init__(self):
        spc.SkinCalibration.__init__(self, '/fabric_skin')

    def raw_data_to_force(self, raw_data):
        d_biased = self.subtract_bias(raw_data, 0)
        calib_data = -d_biased / 50. # calibration!
        return calib_data


if __name__ == '__main__':
    rospy.init_node('fabric_skin_calibration_node')
    
    fsc = Fabric_Skin_Calibration()
    fsc.precompute_taxel_location_and_normal()

    rdc = spc.RawDataClient('/fabric_skin/taxels/raw_data')
    fsc.compute_bias(rdc, 100)

    while not rospy.is_shutdown():
        d = rdc.get_raw_data(True)
        fsc.publish_taxel_array(d)

