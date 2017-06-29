import rospy

import hrl_lib.util as ut

import hrl_fabric_based_tactile_sensor.adc_publisher_node as apn

from m3skin_ros.msg import RawTaxelArray
from geometry_msgs.msg import Transform

from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse
from m3skin_ros.srv import None_String


class Fabric_Skin_Patch():
    def __init__(self):
        self.n_taxels_x = 5
        self.n_taxels_y = 3
        self.taxel_dim_x = 0.05
        self.taxel_dim_y = 0.05

        self.link_name = '/fabric_skin_link'
        self.tar = None_TransformArrayResponse()
        for i in range(self.n_taxels_x):
            for j in range(self.n_taxels_y):
                t = Transform()
                t.translation.x = i*self.taxel_dim_x
                t.translation.y = j*self.taxel_dim_y
                t.translation.z = 0.

                t.rotation.x = 0
                t.rotation.y = 0
                t.rotation.z = 0
                t.rotation.w = 1
                self.tar.data.append(t)

    def local_coord_frames_cb(self, req):
        return self.tar

    def link_name_cb(self, req):
        return self.link_name


if __name__ == '__main__':
    dev_name = '/dev/ttyACM0'
    baudrate = 115200
    serial_dev = apn.setup_serial(dev_name, baudrate)

    raw_data_pub = rospy.Publisher('/fabric_skin/taxels/raw_data',
                                   RawTaxelArray)

    fsp = Fabric_Skin_Patch()
    d = ut.load_pickle('taxel_registration_dict.pkl')
    fsp.link_name = d['tf_name']
    fsp.tar = d['transform_array_response']

    rospy.Service('/fabric_skin/taxels/srv/local_coord_frames',
                  None_TransformArray, fsp.local_coord_frames_cb)
    rospy.Service('/fabric_skin/taxels/srv/link_name', None_String,
                  fsp.link_name_cb)

    rospy.init_node('fabric_skin_driver_node')

    for i in range(10):
        ln = serial_dev.readline()

    rospy.loginfo('Started publishing data')

    rta = RawTaxelArray()
    while not rospy.is_shutdown():
        rta.val_z = apn.get_adc_data(serial_dev)[0:15]
        raw_data_pub.publish(rta)

    serial_dev.close()
