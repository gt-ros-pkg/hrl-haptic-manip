import time
import roslib; roslib.load_manifest('hrl_haptic_manipulation_in_clutter')
import rospy
from geometry_msgs.msg import PointStamped
import numpy as np

if __name__ == '__main__':
    rospy.init_node('goal_updater')
    goal_point_pub = rospy.Publisher('test_goal_update', PointStamped)
    start = time.time()
    
    ps = PointStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = '/world'
    ps.point.x = 0
    ps.point.y = -0.8
    ps.point.z = 0
    
    print "publishing point now"
    while time.time()-start < 100:
        goal_point_pub.publish(ps)
        rospy.sleep(0.01)

    print "all done publishing point, did it work?"
    
    

