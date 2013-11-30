import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('equilibrium_point_control')
import equilibrium_point_control.epc as epc
from hrl_pr2_arms.pr2_arm_darpa_m3 import *
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Bool, Empty

def stop_start_epc():
    # stop current controller
    stop_pub.publish(Bool(True))
    rospy.sleep(0.3)
    # allow controller to start.
    stop_pub.publish(Bool(False))
    rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('debug_pr2_orientation_test')

    robot = PR2Arm('r')

    epcon = epc.EPC(robot)

    while robot.get_joint_angles() == None:
        rospy.sleep(0.1)

    q = robot.get_joint_angles()
    robot.set_ep(q)

    print "joint angles are q:\n", q

    jep = [0.0030335812045541033,
           1.0658363709587833,
           -0.088147778907083293,
           -1.4123731806399289,
           0.66266770567015332,
           -0.099996111038144972,
           -0.70887236899498485]

    epcon.go_jep(jep, speed=math.radians(30.))


    wp_pose_pub = rospy.Publisher('/teleop_rviz/command/way_point_pose', PoseStamped)
    ros_pub = rospy.Publisher('/epc_skin/command/behavior', String)
    stop_pub = rospy.Publisher('/epc/stop', Bool)

    ps = PoseStamped()
    ps.header.frame_id = '/torso_lift_link'

    ps.pose.position.x = 0.727240562439
    ps.pose.position.y = -0.10000000149
    ps.pose.position.z = -0.150012061

    ps.pose.orientation.x = 0.
    ps.pose.orientation.y = 0.
    ps.pose.orientation.z = 0.
    ps.pose.orientation.w = 1.

    i = 0
    while i < 10:
        wp_pose_pub.publish(ps)
        rospy.sleep(0.1)
        i = i+1

    stop_start_epc()

    #ros_pub.publish('go_to_way_point')
    ros_pub.publish('orient_to_way_point')
    rospy.spin()
    #rospy.sleep(0.8)
    #stop_pub.publish(Bool(True))


#this goal and these initial conditions give weird behavior,
#trying to figure out why ...

# poses: 
#   - 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 0
#         nsecs: 0
#       frame_id: /torso_lift_link
#     pose: 
#       position: 
#         x: 0.727240562439
#         y: -0.10000000149
#         z: -0.150012061
#       orientation: 
#         x: 0.0
#         y: 0.0
#         z: 0.0
#         w: 1.0
#     name: way_point

# jep = [0.0030335812045541033,
#        1.0658363709587833,
#        -0.088147778907083293,
#        -1.4123731806399289,
#        0.66266770567015332,
#        -0.099996111038144972,
#        -0.70887236899498485]
