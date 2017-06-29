#!/usr/bin/python

import numpy as np, math
import copy

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.viz as hv
import hrl_lib.util as ut
import rospy
import tf
import hrl_lib.transforms as tr

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from hrl_haptic_manipulation_in_clutter_msgs.msg import TaxelArray
#from m3skin_ros.msg import TaxelArray as TaxelArray_Meka

class VizTaxelArray():

    def __init__(self):

        # for ODE
        #self.use_mobile_base = rospy.get_param('use_mobile_base')
        self.use_mobile_base = False
        
        if self.use_mobile_base:
            try:
                self.tf_lstnr = tf.TransformListener()
            except rospy.ServiceException, e:
                rospy.loginfo("ServiceException caught while instantiating a TF listener. Seems to be normal")            

        _ = rospy.Publisher('/skin/viz/taxel_array', Marker)
        self.marker_pub = rospy.Publisher('/skin/viz/taxel_array_array', MarkerArray)

        rospy.Subscriber('/skin/taxel_array', TaxelArray,
                         self.visualize_taxel_array)
        rospy.Subscriber('/skin/taxel_array_meka', TaxelArray,
                         self.visualize_taxel_array)
            
    def visualize_taxel_array(self, ta):
        
        markers = MarkerArray()
        stamp = ta.header.stamp
        frame = ta.header.frame_id

        if self.use_mobile_base and frame != 'world':
            new_ta = self.transformTaxelArray(ta, 'world')
            ta = new_ta
            stamp = ta.header.stamp
            frame = ta.header.frame_id

        pts = np.column_stack((ta.centers_x, ta.centers_y, ta.centers_z))
        colors = np.zeros((4, pts.shape[0]))
        colors[0,:] = 243/255.0
        colors[1,:] = 132/255.0
        colors[2,:] = 0.
        colors[3,:] = 1.0
        scale = (0.005, 0.005, 0.005)

        duration = 0.
        m = hv.list_marker(pts.T, colors, scale, 'points',
                          frame, duration=duration, m_id=0)
        m.header.stamp = stamp
        markers.markers.append(m)

        # now draw non-zero forces as arrows.
        nrmls = np.column_stack((ta.normals_x, ta.normals_y, ta.normals_z))
        fs = np.column_stack((ta.values_x, ta.values_y, ta.values_z))

        fmags = ut.norm(fs.T).flatten()

        if hasattr(ta, 'link_name'):
            idxs = np.where(fmags > 0.01)[0]
        else:
            # HACK. need to calibrate the skin patch so that something
            # reasonable gets outputted.
            idxs = np.where(fmags > 0.2)[0]

        force_marker_scale = 0.04
        duration = 0.02
        for i in idxs:
            p = np.matrix(pts[i]).T
            n1 = np.matrix(nrmls[i]).T
            n2 = np.matrix(fs[i]).T

            q1 = hv.arrow_direction_to_quat(n1)
            l1 = (n2.T * n1)[0,0] * force_marker_scale


            #if 'electric' not in roslib.__path__[0]:
            #    scale = (l1, 0.2, 0.2)
            #else:
            if 'groovy' not in roslib.__path__[0]:
                scale = (0.2, 0.2, l1)
            else:
                scale = (0.2, 0.02, l1)

            m = hv.single_marker(p, q1, 'arrow', frame, duration=duration,
                                 scale=scale, m_id=3*i+1)
            m.header.stamp = stamp
            markers.markers.append(m)

            q2 = hv.arrow_direction_to_quat(n2)
            l2 = np.linalg.norm(n2) * force_marker_scale

            #if 'electric' not in roslib.__path__[0]:
            #    scale = (l2, 0.2, 0.2)
            #else:
            if 'groovy' not in roslib.__path__[0]:
                scale = (0.2, 0.2, l2)
            else:
                scale = (0.2, 0.02, l2)

            m = hv.single_marker(p, q2, 'arrow', frame, duration=duration,
                                 scale=scale, color=(0.,0.5,0.,1.0),
                                 m_id=3*i+2)
            m.header.stamp = stamp
            markers.markers.append(m)

            m = hv.single_marker(p + n2/np.linalg.norm(n2) * l2 * 1.6, q2, 'text_view_facing', frame,
                                 (0.07, 0.07, 0.07), m_id = 3*i+3,
                                 duration = duration, color=(0.,0.5,0.,1.))
            m.text = '%.1fN'%(np.linalg.norm(n2))
            m.header.stamp = stamp
            markers.markers.append(m)

        self.marker_pub.publish(markers)

    ## Transform a single taxel array message from one frame to another
    # @param ta_msg TaxelArray message object to be transformed
    # @param new_frame The desired frame name
    # @return The transformed message with all values in the new coordinate frame.
    def transformTaxelArray(self, ta_msg, new_frame):   

        # Get the transformation from the desired frame to current frame 
        if ta_msg.header.frame_id == "":
          return ta_msg
        self.tf_lstnr.waitForTransform(new_frame, ta_msg.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
        t1, q1 = self.tf_lstnr.lookupTransform(new_frame, ta_msg.header.frame_id, rospy.Time(0))

        t1 = np.matrix(t1).reshape(3,1)
        r1 = tr.quaternion_to_matrix(q1)

        # Create new message data structure
        new_ta_msg = copy.copy(ta_msg)
        new_ta_msg.header.frame_id = new_frame


        # Perform the transformation
        pts = np.column_stack((ta_msg.centers_x, ta_msg.centers_y, ta_msg.centers_z))
        nrmls = np.column_stack((ta_msg.normals_x, ta_msg.normals_y, ta_msg.normals_z))
        values = np.column_stack((ta_msg.values_x, ta_msg.values_y, ta_msg.values_z))

        pts = r1 * np.matrix(pts).T + t1
        nrmls = r1 * np.matrix(nrmls).T
        values = r1 * np.matrix(values).T

        # Reformat the transformed data to be repackaged as a TaxelArray message
        pts_array = np.asarray(pts)
        nrmls_array = np.asarray(nrmls)
        values_array = np.asarray(values) 

        new_ta_msg.centers_x = pts_array[0, :].tolist()
        new_ta_msg.centers_y = pts_array[1, :].tolist()
        new_ta_msg.centers_z = pts_array[2, :].tolist()

        new_ta_msg.normals_x = nrmls_array[0, :].tolist()
        new_ta_msg.normals_y = nrmls_array[1, :].tolist()
        new_ta_msg.normals_z = nrmls_array[2, :].tolist()

        new_ta_msg.values_x = values_array[0, :].tolist()
        new_ta_msg.values_y = values_array[1, :].tolist()
        new_ta_msg.values_z = values_array[2, :].tolist()

        return new_ta_msg

        
if __name__ == '__main__':
    rospy.init_node('taxel_array_viz_publisher')

    vta = VizTaxelArray()
    
    rospy.loginfo('Started visulizing taxel array!')

    rospy.spin()


