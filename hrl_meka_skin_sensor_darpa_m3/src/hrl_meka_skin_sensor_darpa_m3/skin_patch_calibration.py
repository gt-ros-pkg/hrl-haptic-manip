#!/usr/bin/python

import numpy as np, math
from threading import RLock
import copy

import roslib; roslib.load_manifest('hrl_meka_skin_sensor_darpa_m3')

import rospy
import hrl_lib.util as ut
import hrl_lib.transforms as tr

from m3skin_ros.msg import TaxelArray
from m3skin_ros.msg import RawTaxelArray
from m3skin_ros.srv import None_TransformArray
from m3skin_ros.srv import None_String

from std_msgs.msg import Empty

class RawDataClient():
    def __init__(self, raw_data_topic):
        self.raw_data = None # 1D np array
        self.fresh_data = False
        self.bias = None
        self.lock = RLock()

        rospy.Subscriber(raw_data_topic, RawTaxelArray, self.raw_cb)

    def raw_cb(self, msg):
        with self.lock:
            self.raw_data = np.array(msg.val_z)
            self.fresh_data = True

    def get_raw_data(self, fresh):
        if fresh:
            while not self.fresh_data:
                rospy.sleep(0.002)
        with self.lock:
            self.fresh_data = False
            d = copy.copy(self.raw_data)
        return d


class SkinCalibration():
    def __init__(self):
        self.ta_pub = rospy.Publisher('taxels/forces', TaxelArray)

        self.pos_arr = None # Nx3
        self.nrml_arr = None # Nx3

        self.bias_mn = None
        self.bias_std = None

        self.disable_sensor = False

        srv_nm1 = 'taxels/srv/local_coord_frames'
        srv_nm2 = 'taxels/srv/link_name'

        rospy.loginfo('Waiting for services ...')
        rospy.wait_for_service(srv_nm1)
        rospy.wait_for_service(srv_nm2)
        rospy.loginfo('... Done')

        self.local_coord_frames_srv = rospy.ServiceProxy(srv_nm1,
                                                         None_TransformArray)
        self.link_name_srv = rospy.ServiceProxy(srv_nm2, None_String)
        self.link_name = self.link_name_srv().data

        rospy.Subscriber('disable_sensor', Empty, self.disable_sensor_cb)
        rospy.Subscriber('enable_sensor', Empty, self.enable_sensor_cb)

    def precompute_taxel_location_and_normal(self):
        resp = self.local_coord_frames_srv()
        pos_list = []
        nrml_list = []
        for t in resp.data:
            pos = (t.translation.x, t.translation.y, t.translation.z)
            pos_list.append(pos)
            q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
            rot_mat = tr.quaternion_to_matrix(q)
            nrml_list.append(rot_mat[:,2].A1)

        self.pos_arr = np.array(pos_list)
        self.nrml_arr = np.array(nrml_list)

    def publish_taxel_array(self, raw_data):
        ta = TaxelArray()
        ta.header.frame_id = self.link_name
        ta.header.stamp = rospy.Time.now()

        ta.centers_x = self.pos_arr[:,0]
        ta.centers_y = self.pos_arr[:,1]
        ta.centers_z = self.pos_arr[:,2]

        ta.normals_x = self.nrml_arr[:,0]
        ta.normals_y = self.nrml_arr[:,1]
        ta.normals_z = self.nrml_arr[:,2]

        min_idx = np.argmin(raw_data)
        calib_data = self.raw_data_to_force(raw_data)

        if self.disable_sensor:
            calib_data[:] = 0.

        ta.values_x = ta.normals_x * calib_data
        ta.values_y = ta.normals_y * calib_data
        ta.values_z = ta.normals_z * calib_data

        self.ta_pub.publish(ta)

    # implement this function in classes derived from this one.
    def raw_data_to_force(self, raw_data):
        raise RuntimeError('Unimplemented function')

    # n_std = 0 is the same as no thresholding.
    def subtract_bias(self, data, n_std=0):
        d_biased = data - self.bias_mn
        idxs = np.where(np.abs(d_biased) < n_std * self.bias_std)[0]
        d_biased[idxs] = 0
        return d_biased

    # rdc - raw data client
    def compute_bias(self, rdc, n):
        rospy.loginfo('started bias computation ...')
        d_list = []
        for i in range(n):
            d_list.append(rdc.get_raw_data(fresh = True))

        d_arr = np.row_stack(d_list)
        mn = np.mean(d_arr, 0)
        std = np.std(d_arr, 0)
        self.bias_mn = mn
        self.bias_std = std
        rospy.loginfo('...done')

    def disable_sensor_cb(self, msg):
        self.disable_sensor = True

    def enable_sensor_cb(self, msg):
        self.disable_sensor = False


class SkinCalibration_Naive(SkinCalibration):
    def __init__(self):
        SkinCalibration.__init__(self)

    def raw_data_to_force(self, raw_data):
        meka_taxel_saturate_threshold = 17.
        saturated_taxel_force_value = 80.

        d_biased = self.subtract_bias(raw_data, 6)
        calib_data = d_biased / 1000. # calibration!
        idxs = np.where(calib_data < 1.0)[0]
        calib_data[idxs] = 0.
        idxs = np.where(calib_data > meka_taxel_saturate_threshold)[0]
        calib_data[idxs] = saturated_taxel_force_value
        return calib_data


if __name__ == '__main__':
    rospy.init_node('forearm_raw_data_subscriber')
    
    scn = SkinCalibration_Naive()
    scn.precompute_taxel_location_and_normal()

    rdc = RawDataClient('taxels/raw_data')
    scn.compute_bias(rdc, 1000)

    while not rospy.is_shutdown():
        d = rdc.get_raw_data(True)
        scn.publish_taxel_array(d)

    if False:
        ut.get_keystroke('Hit a key to get biased data')
        d = rdc.get_biased_data(False, 5)
        print 'd:', d








