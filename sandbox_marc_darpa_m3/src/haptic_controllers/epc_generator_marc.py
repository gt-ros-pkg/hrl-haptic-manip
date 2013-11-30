import roslib; roslib.load_manifest('sandbox_marc_darpa_m3')
import equilibrium_point_control.epc as epc
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class EP_Generator_Marc(epc.EP_Generator):
    # @param ep_gen_func: function that returns stop, ea  where ea is the param to the control_function and  stop: string which is '' for epc motion to continue
    def __init__(self, ep_gen_func, control_function,
                 ep_clamp_func=None):
        epc.EP_Generator.__init__(self, ep_gen_func, control_function, ep_clamp_func=None)
        self.goal_pos = None
        self.goal_orientation = None
        rospy.Subscriber('test_goal_update', PoseStamped, self.goal_cb)
    
    def goal_cb(self, data):
        self.goal_pos = np.matrix([data.pose.position.x, 
                                   data.pose.position.y, 
                                   data.pose.position.z]).T
        self.goal_orientation = np.matrix([data.pose.orientation.x, 
                                           data.pose.orientation.y, 
                                           data.pose.orientation.z,
                                           data.pose.orientation.w]).T
    
        # ps = PointStamped()
        # ps.header.stamp = rospy.Time.now()
        # ps.header.frame_id = frame
        # ps.point.x = goal_pos[0,0]
        # ps.point.y = goal_pos[1,0]
        # ps.point.z = goal_pos[2,0]
