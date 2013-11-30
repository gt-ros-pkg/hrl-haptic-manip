import roslib; roslib.load_manifest('sandbox_marc_darpa_m3')
import equilibrium_point_control.epc as epc
import rospy
from geometry_msgs.msg import PointStamped
import numpy as np

class EP_Generator_Marc(epc.EP_Generator):
    # @param ep_gen_func: function that returns stop, ea  where ea is the param to the control_function and  stop: string which is '' for epc motion to continue
    def __init__(self, ep_gen_func, control_function,
                 ep_clamp_func=None):
        epc.EP_Generator.__init__(self, ep_gen_func, control_function, ep_clamp_func=None)
        self.goal_pos = None
        rospy.Subscriber('test_goal_update', PointStamped, self.goal_cb)
    
    def goal_cb(self, data):
        self.goal_pos = np.matrix([data.point.x, data.point.y, data.point.z]).T

        # ps = PointStamped()
        # ps.header.stamp = rospy.Time.now()
        # ps.header.frame_id = frame
        # ps.point.x = goal_pos[0,0]
        # ps.point.y = goal_pos[1,0]
        # ps.point.z = goal_pos[2,0]
