#!/usr/bin/env python

import numpy as np
import optparse

import roslib; 
roslib.load_manifest('hrl_haptic_manipulation_in_clutter_msgs')
roslib.load_manifest('m3skin_ros')
roslib.load_manifest('geometry_msgs')
import rospy
from geometry_msgs.msg import WrenchStamped, Transform
from hrl_haptic_manipulation_in_clutter_msgs.msg import TaxelArray
from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse, None_String

class WrenchToTaxelArray(object):
    def __init__(self, opt=None):
        # Default topic names:
        raw_data_topic = 'wrench_stamped'
        output_data_topic = 'pr2_ft_sensor/taxels/forces'
        scaling_param_name = '~magnitude_scale'

        transform_service_name='/pr2_ft_sensor/taxels/srv/local_coord_frames'
        taxel_string_service_name='/pr2_ft_sensor/taxels/srv/link_name'

        if opt:
          if opt.raw_data_topic:
            raw_data_topic = opt.raw_data_topic
          if opt.output_data_topic:
            output_data_topic = opt.output_data_topic
          if opt.scaling_param_name:
            scaling_param_name = opt.scaling_param_name

        self.wrench_sub = rospy.Subscriber(raw_data_topic, WrenchStamped, self.wrench_stamped_cb)
        self.taxel_array_pub = rospy.Publisher(output_data_topic, TaxelArray,queue_size=1)
        self.scaling = rospy.get_param(scaling_param_name, 1.0)
        self.transform_service  = rospy.Service(transform_service_name, None_TransformArray, self.local_coord_frames_ft_cb)
        self.taxel_string_service = rospy.Service(taxel_string_service_name, None_String, self.link_name_ft_cb)
        self.ft_link_name = self.link_name_ft_cb(None)

    def link_name_ft_cb(self, req):
        return '/l_force_torque_link'

    def local_coord_frames_ft_cb(self, req):
        transform = Transform()
        transform.rotation.w = 1.0
        resp = None_TransformArrayResponse()
        resp.data = [transform]
        return resp

    def wrench_stamped_cb(self, ws):
        """BEWARE DIRTY HACK IN COORDINATE FLIPPING!!!!"""
        force_vec = np.array([ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z])
        scaled_vec = np.multiply(force_vec, self.scaling)
        mag = np.linalg.norm(force_vec)
        normalized_vec = np.divide(force_vec,mag)

        ta = TaxelArray()
        ta.header.frame_id = '/l_force_torque_link' #self.ft_link_name
        ta.header.stamp = rospy.Time.now()
        ta.sensor_type = 'force'
        ta.link_names = ['wrist_roll']
        ta.centers_x = [0.]
        ta.centers_y = [0.]
        ta.centers_z = [0.]
        ta.normals_x = [-normalized_vec[0]]
        ta.normals_y = [-normalized_vec[1]]
        ta.normals_z = [-normalized_vec[2]]
        ta.values_x = [-scaled_vec[0]]
        ta.values_y = [-scaled_vec[1]]
        ta.values_z = [-scaled_vec[2]]

        self.taxel_array_pub.publish(ta)

if __name__=='__main__':
    p = optparse.OptionParser()
    p.add_option('-i', '--input', dest="raw_data_topic", help="Raw data input topic. Default: \'wrench_stamped\'")
    p.add_option('-o', '--output', dest="output_data_topic", help="Processed data output topic (TaxelArray). Default: \'pr2_ft_sensor/taxels/forces\'")
    p.add_option('-s', '--scaling', dest="scaling_param_name", help="Scaling parameter name. Default: \'~magnitude_scale\'")

    (opt,args) = p.parse_args()

    rospy.init_node('ft_to_taxel_array_node')
    rospy.loginfo("Initialising FT to TaxelArray Node")
    wrench_to_taxel = WrenchToTaxelArray(opt)
    rospy.loginfo("Running FT to TaxelArray converter")
    rospy.spin()
