#!/usr/bin/env python

import sys
import copy
import cPickle as pkl
import argparse

import roslib; roslib.load_manifest('hrl_dynamic_mpc')
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import darci_arm_kinematics as dak


class Recorder(object):
    def __init__(self, save_id):
        self.save_id = save_id
        self.sub = rospy.Subscriber('humanoid_state', JointState, self.state_cb)
        self.joint_angles = None
        self.kinematics = dak.DarciArmKinematics('l')

    def state_cb(self, jsm):
        self.joint_angles = copy.copy(jsm.position[7:])

    def record_current_ee_pose(self):
        while self.joint_angles is None and not rospy.is_shutdown():
            print "Waiting for joint angles"
            rospy.sleep(1)
        raw_input("Setup complete.  Press 'Enter' to record pose.")
        pos, rot = self.kinematics.FK(self.joint_angles)
        pos = pos.getA1().tolist()
        ps = PoseStamped()
        ps.header.frame_id = 'torso_lift_link'
        ps.header.stamp = rospy.Time.now()
        ps.pose.position = Point(*pos)
        ps.pose.orientation = Quaternion(0,0,0,1)

        with open('demo_pose_'+self.save_id+'.pkl', 'wb') as f:
            pkl.dump(ps, f, pkl.HIGHEST_PROTOCOL)
        print "Pose %s Written to file" %self.save_id

def publish_pose(topic, pose_id):
    pub = rospy.Publisher(topic, PoseStamped)
    rospy.sleep(2)
    with open('demo_pose_'+pose_id+'.pkl', 'rb') as f:
        pose = pkl.load(f)
    pub.publish(pose)
    print "Publishing pose: %s to topic: %s" %(pose_id, topic)

if __name__=="__main__":
    rospy.init_node('pose_assist')

    p = argparse.ArgumentParser(description="ROS Node for getting/sending goal posed for Darpa final dmeo.",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('rec_pub', action='store', type=str,
            choices=['record','r', 'rec', 'publish','pub', 'p'],
                    help = 'indicate whether to record or publish a ppose')
    p.add_argument('--topic', '-t', action="store", type=str, dest='topic', default="demo/goal_pose",
                   help="topic to publish PoseStamped msg to.")
    p.add_argument('--id', action="store", type=str, dest='pose_id',
                   help="id to save PoseStamped msg to, or which pose to publish.")
    args = p.parse_args()
    if 'r' in args.rec_pub:
        if args.pose_id is None:
            print "Must indicate id to save pose in order to record"
            sys.exit()
        else:
            recorder = Recorder(args.pose_id)
            recorder.record_current_ee_pose()
    elif 'p' in args.rec_pub:
        if args.pose_id is None:
            print "Must indicate a pose to publish"
            sys.exit()
        publish_pose(args.topic, args.pose_id)
