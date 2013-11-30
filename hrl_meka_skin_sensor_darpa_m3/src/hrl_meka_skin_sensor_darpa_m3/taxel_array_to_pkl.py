


import roslib; roslib.load_manifest('darpa_m3')
import rospy
import hrl_lib.util as ut

from m3skin_ros.msg import TaxelArray as TaxelArray_Meka


def taxel_array_cb(ta):
    ut.save_pickle(ta, 'taxel_array.pkl')
    rospy.loginfo('Saved TaxelArray_Meka into a pkl')


if __name__ == '__main__':

    saved_taxel_array = False

    rospy.init_node('taxel_array_to_pkl')

    taxel_topic = '/skin_patch_forearm_right/taxels/forces'
    rospy.Subscriber(taxel_topic, TaxelArray_Meka,
                     taxel_array_cb)

    rospy.spin()




