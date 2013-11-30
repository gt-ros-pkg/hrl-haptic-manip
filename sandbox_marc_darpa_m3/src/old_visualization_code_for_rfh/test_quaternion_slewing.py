import math, numpy as np
import sys

import interactive_marker_util as imu

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy

import hrl_lib.transforms as tr
import hrl_lib.util as ut

import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh

from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerFeedback, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Bool, Empty



rospy.init_node('test_slewing_of_orientation')

ps = PointStamped()
ps.header.frame_id = '/torso_lift_link'

ps.point.x = 0.0
ps.point.y = 0.0
ps.point.z = 0.0

#wp_im = imu.make_6dof_pr2_gripper(False, ps, 0.28, (1., 1., 0.,0.4))

# q_start =[-0.63631903, -0.01295735,  0.1602617,   0.75448418]
# q_end=[-0.9761871,  -0.12442427, -0.01664095, -0.17691927]
# q_end = tr.tft.random_quaternion() 
# q_start = tr.tft.random_quaternion()
#q_end = [ 0.58742026,  0.30940073,  0.71895076,  0.20571444]
#q_end = [ 0.31675892, -0.22651806,  0.74920529,  0.5357656 ]
#q_end = [ 0.69613524,  0.,  0., 0.71791067]
q_end=[-0.5017, -0.492,   0.4981,  0.508 ]
q_start = [ 0.4034,  0.8143,  0.1956, -0.3686]


rot_end = tr.quaternion_to_matrix(q_end)
rot_x = tr.rotX(math.radians(180))
#rot_new = rot_end*rot_x*rot_end.T
rot_new = rot_end*rot_x
q_end_prime = tr.matrix_to_quaternion(rot_new)

end_diff = ut.quat_angle(q_start, q_end)
end_prime_diff = ut.quat_angle(q_start, q_end_prime)

if abs(end_prime_diff) < abs(end_diff):
    q_end = q_end_prime

array_pub = rospy.Publisher('/slewing_test/visualization_marker_array', MarkerArray)
array_start_pub = rospy.Publisher('/slewing_test_start/visualization_marker_array', MarkerArray)
array_end_pub = rospy.Publisher('/slewing_test_end/visualization_marker_array', MarkerArray)

m_array_start = MarkerArray()
m_array_start = imu.make_pr2_gripper_marker(ps, 
                                            [1., 0., 0., 0.4], 
                                            q_start,
                                            marker_array=m_array_start, 
                                            mesh_id_start = 300)

rospy.loginfo('made the start marker')

i = 0
while i < 100:
    array_start_pub.publish(m_array_start)
    i = i+1
    rospy.sleep(0.01)

rospy.loginfo('finished publishing the start marker')

m_array_end = MarkerArray()
m_array_end = imu.make_pr2_gripper_marker(ps, 
                                          [0, 1, 0, 0.4], 
                                          q_end,
                                          marker_array=m_array_end,
                                          mesh_id_start = 200)

rospy.loginfo('made the end marker')

i = 0
while i < 100:
    array_end_pub.publish(m_array_end)
    i = i+1
    rospy.sleep(0.01)

rospy.loginfo('finished publishing the end marker')

# for i in xrange(1500):
#     m_array = MarkerArray()
#     step = i*2./3000.
#     interp_q_goal = tr.tft.quaternion_slerp(q_start, q_end, step)
#     m_array = imu.make_pr2_gripper_marker(ps, 
#                                         [1, 1, 1, 0.4], 
#                                         interp_q_goal,
#                                         marker_array=m_array)
#     array_pub.publish(m_array)
#     rospy.sleep(0.01)

m_array = MarkerArray()
count = 20.
for i in xrange(int(count)):
    step = i*1./count
    interp_q_goal = tr.tft.quaternion_slerp(q_start, q_end, step)
    m_array = imu.make_pr2_gripper_marker(ps, 
                                        [0.03, 0, 1, 0.4], 
                                        interp_q_goal,
                                        marker_array=m_array,
                                        mesh_id_start=1000+i*30)

array_pub.publish(m_array)
rospy.sleep(10)
print "q_start :\n", q_start
print "q_end :\n", q_end




#examples

#closer rotation due to symmetry of gripper
# q_start :
# [ 0.73899734 -0.40863547  0.53379865  0.04426044]
# q_end :
# [ 0.54018783  0.46848908  0.69509199  0.07458029]

#BEST ONE TO TEST ROTATION ON 
# q_start :
# [-0.63631903 -0.01295735  0.1602617   0.75448418]
# q_end :
# [-0.9761871  -0.12442427 -0.01664095 -0.17691927]

