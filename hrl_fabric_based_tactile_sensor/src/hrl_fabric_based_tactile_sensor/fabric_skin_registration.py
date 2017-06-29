import math
import numpy as np
from enthought.mayavi import mlab

import rospy

import hrl_lib.transforms as tr
import hrl_lib.util as ut

import hrl_meka_skin_sensor_darpa_m3.skin_patch_calibration as spc
from m3skin_ros.srv import None_TransformArrayResponse
from geometry_msgs.msg import Transform


def plot_taxel_registration_dict(d, color):
    tar = d['transform_array_response']
    x_l, y_l, z_l = [], [], []
    for t in tar.data:
        x_l.append(t.translation.x)
        y_l.append(t.translation.y)
        z_l.append(t.translation.z)

    mlab.points3d(x_l, y_l, z_l, mode='sphere', color=color, scale_factor=0.002)


# rdc - raw data client
def compute_bias(rdc, n):
    rospy.loginfo('started bias computation ...')
    d_list = []
    for i in range(n):
        d_list.append(rdc.get_raw_data(fresh=True))

    d_arr = np.row_stack(d_list)
    mn = np.mean(d_arr, 0)
    std = np.std(d_arr, 0)
    bias_mn = mn
    bias_std = std
    rospy.loginfo('...done')

    return bias_mn, bias_std


if __name__ == '__main__':

    if False:
        d = ut.load_pickle('taxel_registration_dict.pkl')
        plot_taxel_registration_dict(d, (0, 0, 1))
        d = ut.load_pickle('bah.pkl')
        plot_taxel_registration_dict(d, (0, 1, 0))
        mlab.show()

    if True:
        # assuming taxels distributed on a cylinder with the z-axis of the
        # TF frame corresponding to the axis of the cylinder.
        n_circum = 5
        n_axis = 3
        tf_link_name = '/wrist_LEFT'

        rad = 0.04
        dist_along_axis = 0.04
        angle_along_circum = 2*math.pi / n_circum

        offset_along_axis = 0.02
        offset_along_circum = math.radians(0)

        n_taxels = n_circum * n_axis

        # dummy values
        tar = None_TransformArrayResponse()
        for i in range(n_taxels):
            t = Transform()
            t.translation.x = i*dist_along_axis
            t.translation.y = i*dist_along_axis
            t.translation.z = 0.

            t.rotation.x = 0
            t.rotation.y = 0
            t.rotation.z = 0
            t.rotation.w = 1
            tar.data.append(t)

        rospy.init_node('fabric_skin_registration_node')

        rdc = spc.RawDataClient('/fabric_skin/taxels/raw_data')
        bias_mn, bias_std = compute_bias(rdc, 20)

        for i in range(n_axis):
            for j in range(n_circum):
                raw_input('Press on taxel and hit ENTER')
                dat = rdc.get_raw_data(True)
                min_idx = np.argmin(dat-bias_mn)
                ang = j*angle_along_circum + offset_along_circum
                x = rad * math.cos(ang)
                y = rad * math.sin(ang)
                z = offset_along_axis + i * dist_along_axis

                rot_mat = tr.Rz(-ang)*tr.Ry(math.radians(-90))
                quat = tr.matrix_to_quaternion(rot_mat)

                tar.data[min_idx].translation.x = x
                tar.data[min_idx].translation.y = y
                tar.data[min_idx].translation.z = z

                tar.data[min_idx].rotation.x = quat[0]
                tar.data[min_idx].rotation.y = quat[1]
                tar.data[min_idx].rotation.z = quat[2]
                tar.data[min_idx].rotation.w = quat[3]

        d = {}
        d['tf_name'] = tf_link_name
        d['transform_array_response'] = tar
        ut.save_pickle(d, 'taxel_registration_dict.pkl')
