#!/usr/bin/env python

from threading import Thread, Lock, RLock
import numpy as np
import copy
import sys
import cPickle as pkl
import time

import roslib; roslib.load_manifest('hrl_dynamic_mpc')
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hrl_lib import transforms as tr
from hrl_msgs.msg import FloatArrayBare
from hrl_haptic_manipulation_in_clutter_srvs.srv import ServiceLIC, ServiceLICRequest, EnableHapticMPC, EnableHapticMPCRequest

class GoalStatus(object):
    LIC_1_REQ = 0
    LIC_1_SETUP = 1
    LIC_1_REACH = 2
    LIC_1_RETREAT = 3
    LIC_2_REQ = 4
    LIC_2_SETUP = 5
    LIC_2_REACH = 6
    PLAN_REQUEST = 7
    PLAN_REACH = 8
    POST_PLAN_REACH = 9
    PLAN_PULLBACK = 10
    def __init__(self, goal_id, goal_pose):
        self.goal_id = goal_id
        self.goal_pose = goal_pose
        self.complete = False
        self.succeeded = False
        self.final_step = None
        self.total_time = 0.
        self.times = {}

    def update(self, complete=None, succeeded=None, final_step=None, total_time=None):
        if complete is not None:
            self.complete = complete
        if succeeded is not None:
            self.succeeded = succeeded
        if final_step is not None:
            assert final_step in range(11), "Unknown value for final step in goal status %s" %self.goal_id
            self.final_step = final_step
        if total_time is not None:
            self.total_time = total_time


def get_time_stamp():
    lt = time.localtime()
    return '_'.join(map(str, [lt.tm_year, lt.tm_mon, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec]))

def handle_ros_interrupt(func):
    def inner(self):
        try:
            return func(self)
        except rospy.ROSInterruptException:
            rospy.loginfo("[%s] Stopping on ROS Interrupt" %self.name)
    return inner

class StateRecorder(Thread):
    POS_DIFF_THRESH = 0.05
    ANGLE_DIFF_THRESH = np.radians(10)
    def __init__(self, robot_client, update_rate=100, name="State Recorder"):
        super(StateRecorder, self).__init__() #Thread init
        self.robot_client = robot_client
        self.update_rate = rospy.Rate(update_rate)
        self.name = name
        self.record_flag = False
        self.traj_lock = Lock()
        self.pose_lock = Lock()
        self.trajectory_buffer = []
        self.pose_buffer = []
        self.daemon = True
        self.start()
        rospy.loginfo("[%s] Started." %self.name)

    def clear(self):
        self.clear_trajectory_buffer()
        self.clear_pose_buffer()

    def clear_trajectory_buffer(self):
        with self.traj_lock:
            self.trajectory_buffer = []

    def clear_pose_buffer(self):
        with self.pose_lock:
            self.pose_buffer = []

    def diff_pose(self, p1, p2):
        if np.linalg.norm(np.subtract(p1, p2)) > self.POS_DIFF_THRESH:
            return True
        return False

    def diff_configs(self, cfg1, cfg2):
        if np.max(np.abs(np.subtract(cfg1, cfg2))) > self.ANGLE_DIFF_THRESH:
            return True
        return False

    def record(self):
        self.record_flag = True

    def pause(self):
        self.record_flag = False

    def start_recording(self):
        self.clear()
        self.record()

    def stop(self):
        self.pause()
        self.clear()

    def record_current_pose(self):
        current_config = self.robot_client.get_joint_angles()
        current_ee_pos, current_ee_rot = self.robot_client.kinematics.FK(current_config)
        self.trajectory_buffer.append(current_config)
        self.pose_buffer.append(current_ee_pos.getA1().tolist())

    def get_pose_history(self):
        with self.pose_lock:
            hist = copy.copy(self.pose_buffer)
            return hist

    def get_trajectory_history(self):
        with self.traj_lock:
            hist = copy.copy(self.trajectory_buffer)
            return hist

    @handle_ros_interrupt
    def run(self):
        while not rospy.is_shutdown():
            self.update_rate.sleep()
            if not self.record_flag:
                continue
            current_config = self.robot_client.get_joint_angles()
            current_ee_pos, current_ee_rot = self.robot_client.kinematics.FK(current_config)
            current_ee_pos = current_ee_pos.getA1().tolist()

            if not self.trajectory_buffer:
                self.trajectory_buffer.append(current_config)
                continue
            last_config = self.trajectory_buffer[-1]
            if self.diff_configs(current_config, last_config):
                self.trajectory_buffer.append(current_config)

            if not self.pose_buffer:
                self.pose_buffer.append(current_ee_pos)
                continue
            last_pose = self.pose_buffer[-1]
            if self.diff_pose(current_ee_pos, last_pose):
                self.pose_buffer.append(current_ee_pos)

class TrajectoryManager(Thread):
    def __init__(self, robot_client, timeout=5, update_rate=100, name="Trajectory Manager"):
        super(TrajectoryManager, self).__init__() #Thread init
        self.robot_client = robot_client
        self.timeout = rospy.Duration(timeout)
        self.update_rate = rospy.Rate(update_rate)
        self.name = name
        self.controller_posture_pub = rospy.Publisher('haptic_mpc/goal_posture', FloatArrayBare)
        self.traj_lock = RLock()
        self.trajectory = None
        self.current_ind = 0
        self.last_best_err = None
        self.last_move_time = rospy.Time.now()
        self.paused = False
        self.succeeded = False
        self.failed = False
        self.max_angle_success_thresh = np.radians(5)
        self.mean_angle_success_thresh = np.radians(3)
        self.max_angle_fail_thresh = np.radians(30)
        self.mean_angle_fail_thresh = np.radians(25)
        self.run_start = rospy.Time.now()
        self.run_duration = rospy.Duration(0)
        self.daemon = True
        self.start()
        rospy.loginfo("[%s] Started" %self.name)

    def set_fail_thresholds(self, max_angle=None, mean_angle=None):
        if max_angle is not None:
            self.max_angle_fail_thresh = max_angle
        if mean_angle is not None:
            self.mean_angle_fail_thresh = mean_angle

    def set_success_thresholds(self, max_angle=None, mean_angle=None):
        if max_angle is not None:
            self.max_angle_success_thresh = np.radians(max_angle)
        if mean_angle is not None:
            self.mean_angle_success_thresh = np.radians(mean_angle)

    def joint_config_dist(self, q1, q2):
        return np.abs(np.subtract(q1, q2))

    def near_joint_config(self, joint_config):
        current_q = self.robot_client.get_joint_angles()
        err = self.joint_config_dist(joint_config, current_q)
        if (np.max(err) < self.max_angle_success_thresh and
            np.mean(err) < self.mean_angle_success_thresh):
            self.last_best_err = None
            self.last_move_time = rospy.Time.now()
            print "Successful Error: %s" %np.degrees(err)
            return True
        return False

    def progressing(self, joint_config):
        current_q = self.robot_client.get_joint_angles()
        diff = self.joint_config_dist(joint_config, current_q)
        err = np.linalg.norm(self.joint_config_dist(joint_config, current_q))
#        rospy.loginfo("[%s] Goal: %s" %(self.name, np.degrees(joint_config)))
#        rospy.loginfo("[%s] State:%s" %(self.name, np.degrees(current_q)))
#        rospy.loginfo("[%s] Error:%s" %(self.name, np.degrees(diff)))
        if self.last_best_err is None:
            self.last_best_err = err
        if err <= 0.95 * self.last_best_err:
            self.last_move_time = rospy.Time.now()
            self.last_best_err = err
            return True
        elif rospy.Time.now() - self.last_move_time < self.timeout:
            return True
        return False

    def check_failure(self, joint_config):
        current_q = self.robot_client.getJointAngles()
        err = self.joint_config_dist(joint_config, current_q)
        if (np.max(err) > self.max_angle_fail_thresh or
            np.mean(err) > self.mean_angle_fail_thresh):
            rospy.loginfo("[%s] Error from Trajectory too great." %self.name)
            return True
        return False

    def get_current_goal(self):
        return self.trajectory[self.current_ind].positions

    def get_run_duration(self):
        dur = copy.copy(self.run_duration)
        self.run_duration = rospy.Duration(0)
        return dur

    def set_trajectory(self, trajectory):
        assert isinstance(trajectory, JointTrajectory), "Attempted to set the trajectory to something other than a JointTrajectory Msg"
        rospy.loginfo("[%s] Received new trajectory." %self.name)
        self.cancel()
        with self.traj_lock:
            self.trajectory = trajectory.points
            self.last_move_time = rospy.Time.now()
        if self.check_failure(self.trajectory[self.current_ind].positions):
            self.cancel()
            self.failed = True
            rospy.loginfo("[%s] Arm too far from beginning of trajectory. Aborting." %self.name)
        else:
            self.run_start = rospy.Time.now()
            msg = FloatArrayBare()
            msg.data = self.trajectory[0].positions
#            print "[%s] Sending Posture Goal: %s" %(self.name, msg.data)
            self.controller_posture_pub.publish(msg)

    def cancel(self):
        with self.traj_lock:
            self.trajectory = None
            self.current_ind = 0
            self.last_best_err = None
            self.failed = False
            self.succeeded = False

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    @handle_ros_interrupt
    def run(self, trajectory=None):
        #TODO: Add trajectory-reduction based on inflection points
#        self.last_loop_time = rospy.Time.now()
        while not rospy.is_shutdown():
            #now = rospy.Time.now()
            #print rospy.loginfo("[%s] Rate: %.2f" %(self.name, (now-self.last_loop_time).to_sec()))
 #           self.last_loop_time  = now
            self.update_rate.sleep()
            if self.paused:
                continue
            with self.traj_lock:
                #check for a trajectory to send.  If none, wait.
                if self.trajectory is None:
                    self.current_ind = 0
                    continue
                #rospy.loginfo("[%s] Processing new goal." %self.name) 
                current_goal = copy.copy(self.trajectory[self.current_ind].positions)
                if not self.progressing(current_goal):
                    rospy.loginfo("[%s] Failed to progress along trajectory." %self.name)
                    self.cancel()
                    self.failed = True
                    continue
                if self.near_joint_config(current_goal):
                    self.current_ind += 1
                    self.last_move_time = rospy.Time.now()
                    #check if at end of trajectory.  If finished, reset.
                    if self.current_ind >= len(self.trajectory):
                        rospy.loginfo("[%s] Successfully finished tracking trajectory." %self.name)
                        self.cancel()
                        self.succeeded = True
                        self.run_duration = rospy.Time.now() - self.run_start
                        continue
                    msg = FloatArrayBare()
                    msg.data = copy.copy(self.trajectory[self.current_ind].positions)
#                    print "[%s] Sending Posture Goal: %s" %(self.name, msg.data)
                    self.controller_posture_pub.publish(msg)
                elif self.check_failure(self.trajectory[self.current_ind].positions):
                    rospy.loginfo("[%s] Faiured to track trajectory." %self.name)
                    self.cancel()
                    self.failed = True

class TrajectoryRequester(object):
    def __init__(self, name="Trajectory Requester", timeout=180.):
        self.name = name
        self.timeout = rospy.Duration(timeout)
        self.trajectory = None
        self.lock = Lock()
        self.succeeded = False
        self.failed = False
        self.ignore_return = False
        self.planner_goal_pub = rospy.Publisher('hrl_planner/goal_pose', PoseStamped)
        self.planner_traj_sub = rospy.Subscriber('hrl_planner/joint_trajectory', JointTrajectory, self.planner_traj_cb)
        self.timer = None
        self.run_start = rospy.Time.now()
        self.run_duration = rospy.Duration(0)
        rospy.loginfo("[%s] Started." %self.name)

    def start_timer(self):
        def timeout_cb(timeout_event):
            rospy.loginfo("[%s] Trajectory Request timed out after %f seconds." %(self.name, self.timeout.to_sec()))
            self.cancel()
            self.failed = True
            self.run_duration = rospy.Time.now() - self.run_start
        self.timer = rospy.Timer(self.timeout, timeout_cb, oneshot=True)

    def set_goal(self, ps):
        assert isinstance(ps, PoseStamped), "Tried to pass a non-PoseStamped goal to %s.\r\n %s" %(self.name, ps)
        rospy.loginfo("[%s] Requesting path from planner." %self.name)
        self.cancel()
        self.ignore_return = False
        self.run_start = rospy.Time.now()
        self.planner_goal_pub.publish(ps)
        self.start_timer()

    def planner_traj_cb(self, traj_msg):
        rospy.loginfo("[%s] Received Trajectory Msg from Planner."  %self.name)
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
        else:
            rospy.loginfo("[%s] New Trajectory Received, but no timer was active..." %self.name)
        self.run_duration = rospy.Time.now() - self.run_start
        if self.ignore_return:
            self.cancel()
            return
        self.cancel()
        if not traj_msg.points:
            rospy.loginfo("[%s] Received Empty Trajectrory: Planner Failed." %self.name)
            self.failed = True
        else:
            rospy.loginfo("[%s] Received Trajectrory from Planner." %self.name)
            with self.lock:
                self.trajectory = copy.copy(traj_msg)
            self.succeeded = True

    def cancel(self):
        with self.lock:
            self.succeeded = False
            self.failed = False
            self.ignore_return = True
            self.trajectory = None

    def get_trajectory(self):
        with self.lock:
            traj = copy.copy(self.trajectory)
        self.cancel()
        return traj

    def get_run_duration(self):
        dur = copy.copy(self.run_duration)
        self.run_duration = rospy.Duration(0)
        return dur

class ReachManager(Thread):
    def __init__(self, robot_client, update_rate=100, name="Reach Manager", timeout=5):
        super(ReachManager, self).__init__()
        self.robot_client = robot_client
        self.update_rate = rospy.Rate(update_rate)
        self.name = name
        self.timeout = rospy.Duration(timeout)
        self.lock = Lock()
        self.goal_msg = None
        self.goal_pos = []
        self.controller_goal_pub = rospy.Publisher('haptic_mpc/goal_pose', PoseStamped)
        self.last_move_time = rospy.Time.now()
        self.last_best_distance = None
        self.succeeded = False
        self.failed = False
        self.at_goal_thresh = 0.01
        self.run_start = rospy.Time.now()
        self.run_duration = rospy.Duration(0)
        self.daemon = True
        self.start()
        rospy.loginfo("[%s] Started." %self.name)

    def set_goal(self, ps):
        rospy.loginfo("[%s] Received New Goal." %self.name)
        self.cancel()
        with self.lock:
            self.goal_msg = ps
            self.goal_pos = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
            curr_pos, curr_rot = self.robot_client.kinematics.FK(self.robot_client.get_joint_angles())
            self.last_move_time = rospy.Time.now()
            self.last_best_distance = np.linalg.norm(np.subtract(curr_pos.getA1().tolist(), self.goal_pos))
        self.run_start = rospy.Time.now()
        self.controller_goal_pub.publish(ps)

    def set_goal_thresh(self, dist):
        self.at_goal_thresh = dist

    def set_timeout(self, duration):
        self.timeout = rospy.Duration(duration)

    def get_run_duration(self):
        dur = copy.copy(self.run_duration)
        self.run_duration = rospy.Duration(0)
        return dur

    def cancel(self):
        with self.lock:
            self.goal_msg = None
            self.goal_pos = []
            self.succeeded = False
            self.failed = False

    def at_goal(self):
        curr_pos, curr_rot = self.robot_client.kinematics.FK(self.robot_client.get_joint_angles())
        with self.lock:
            err = np.linalg.norm(np.subtract(curr_pos.getA1().tolist(), self.goal_pos))
            if err < self.at_goal_thresh:
                return True
            return False

    def progressing(self):
        curr_pos, curr_rot = self.robot_client.kinematics.FK(self.robot_client.get_joint_angles())
        with self.lock:
            dist = np.linalg.norm(np.subtract(curr_pos.getA1().tolist(), self.goal_pos))
            if dist < 0.95 * self.last_best_distance:
                self.last_move_time = rospy.Time.now()
                self.last_best_distance = dist
                return True
            elif rospy.Time.now() - self.last_move_time < self.timeout:
                return True
        return False

    @handle_ros_interrupt
    def run(self):
        while not rospy.is_shutdown():
            self.update_rate.sleep()
            if not self.goal_pos:
                continue
            if self.at_goal():
                rospy.loginfo("[%s] At Goal." %self.name)
                self.cancel()
                self.succeeded = True
                self.run_duration = rospy.Time.now() - self.run_start
                continue
            if not self.progressing():
                rospy.loginfo("[%s] Not Progressing." %self.name)
                self.cancel()
                self.failed = True
                self.run_duration = rospy.Time.now() - self.run_start

class LICFirstRequester(Thread):
    def __init__(self, update_rate=100, name="LIC First Requester"):
        super(LICFirstRequester, self).__init__()
        self.LIC_client = rospy.ServiceProxy('hrl_lic/lic_1st', ServiceLIC)
        self.update_rate = rospy.Rate(update_rate)
        self.name = name
        self.lock = Lock()
        self.goal = None
        self.start_config = None
        self.succeeded = False
        self.failed = False
        self.ignore_return = False
        rospy.loginfo("[%s] Checking for LIC Service" %self.name)
        try:
            self.LIC_client.wait_for_service(1)
        except rospy.ROSException as e:
            rospy.logwarn("[%s] Could not find LIC service. Shutting Down." %self.name)
            return
        self.run_start = rospy.Time.now()
        self.run_duration = rospy.Duration(0)
        self.daemon = True
        self.start()
        rospy.loginfo("[%s] Started." %self.name)

    def cancel(self):
        with self.lock:
            self.goal = None
            self.failed = False
            self.succeeded = False
            self.ignore_return = True

    def set_goal(self, ps):
        rospy.loginfo("[%s] Received new goal." %self.name)
        assert isinstance(ps, PoseStamped), "Attempted to send non-PoseStamped data to %s" %self.name
        self.cancel()
        self.ignore_return = False
        with self.lock:
            self.run_start = rospy.Time.now()
            self.goal = ps

    def get_config(self):
        with self.lock:
            cfg = copy.copy(self.start_config)
            self.start_config = None
        return cfg

    def get_run_duration(self):
        dur = copy.copy(self.run_duration)
        self.run_duration = rospy.Duration(0)
        return dur

    @handle_ros_interrupt
    def run(self):
        while not rospy.is_shutdown():
            self.update_rate.sleep()
            if self.goal is not None:
                lic_req = ServiceLICRequest()
                try:
                    with self.lock:
                        lic_req.goal_pose = copy.copy(self.goal)
                        self.start_config = self.LIC_client.call(lic_req).joint_angles.data
                        rospy.loginfo("[%s] Received Start Config: %s" %(self.name, np.degrees(self.start_config)))
                except rospy.service.ServiceException as e:
                    self.cancel()
                    self.failed = True
                    rospy.logerr("[%s] Service Exception when calling LIC Service: \r\n\t %s" %(self.name, e.message))
                    continue
                finally:
                    self.run_duration = rospy.Time.now() - self.run_start
                if self.ignore_return:
                    self.cancel()
                    self.start_config = None
                    continue
                self.cancel()
                self.succeeded = True
                rospy.loginfo("[%s] Received Reach Configuration." %self.name)

class LICSecondRequester(Thread):
    def __init__(self, update_rate=100, name="LIC Second Requester"):
        super(LICSecondRequester, self).__init__()
        self.LIC_client = rospy.ServiceProxy('hrl_lic/lic_2nd', ServiceLIC)
        self.update_rate = rospy.Rate(update_rate)
        self.name = name
        self.lock = Lock()
        self.goal = None
        self.first_start_pose = None
        self.first_end_pose = None
        self.start_config = None
        self.succeeded = False
        self.failed = False
        self.ignore_return = False
        rospy.loginfo("[%s] Checking for LIC Service" %self.name)
        try:
            self.LIC_client.wait_for_service(1)
        except rospy.ROSException as e:
            rospy.logwarn("[%s] Could not find LIC service. Shutting Down." %self.name)
            return
        self.run_start = rospy.Time.now()
        self.run_duration = rospy.Duration(0)
        self.daemon = True
        self.start()
        rospy.loginfo("[%s] Started." %self.name)

    def cancel(self):
        with self.lock:
            self.goal = None
            self.first_start_pose = None
            self.first_end_pose = None
            self.failed = False
            self.succeeded = False
            self.ignore_return = True

    def set_goal(self, ps, prev_start, prev_end):
        assert isinstance(ps, PoseStamped), "Attempted to send non-PoseStamped data to %s" %self.name
        assert isinstance(prev_start, PoseStamped), "Attempted to send non-PoseStamped data to %s for first reach endpoint" %self.name
        rospy.loginfo("[%s] Received new goal." %self.name)
        self.cancel()
        self.ignore_return = False
        with self.lock:
            self.run_start = rospy.Time.now()
            self.goal = copy.copy(ps)
            self.first_start_pose = copy.copy(prev_start)
            self.first_end_pose = copy.copy(prev_end)

    def get_config(self):
        with self.lock:
            cfg = copy.copy(self.start_config)
            self.start_config = None
        return cfg

    def get_run_duration(self):
        dur = copy.copy(self.run_duration)
        self.run_duration = rospy.Duration(0)
        return dur

    @handle_ros_interrupt
    def run(self):
        while not rospy.is_shutdown():
            self.update_rate.sleep()
            if (self.goal is not None and
                self.first_start_pose is not None and
                self.first_end_pose is not None):

                lic_req = ServiceLICRequest()
                lic_req.features = FloatArrayBare()
                with self.lock:
                    lic_req.goal_pose = copy.copy(self.goal)
                    lic_req.last_start_pose = copy.copy(self.first_start_pose)
                    lic_req.features.data = copy.copy(self.first_end_pose)
                try:
                    with self.lock:
                        self.start_config = self.LIC_client.call(lic_req).joint_angles.data
                        rospy.loginfo("[%s] Received Start Config: %s" %(self.name, np.degrees(self.start_config)))
                except rospy.service.ServiceException as e:
                    self.cancel()
                    self.failed = True
                    rospy.logerr("[%s] Service Exception when calling LIC service: \r\n\t %s" %(self.name, e.message))
                    continue
                finally:
                    self.run_duration = rospy.Time.now() - self.run_start
                if self.ignore_return:
                    self.cancel()
                    with self.lock:
                        self.start_config = None
                    continue
                self.cancel()
                self.succeeded = True
                rospy.loginfo("[%s] Received Reach Configuration." %self.name)

class WaitProcess(object):
    def __init__(self, name="Waiting"):
        self.name = name
        self.succeeded = False
        self.failed = False

    def cancel(self):
        """ Intentially empty."""
        pass

class RobotManager(object):
    def __init__(self, robot_client, start_config=None, record=False, update_rate=100, name="Robot Manager"):
        self.robot_client = robot_client
        if start_config is None:
            self.start_config = self.robot_client.get_joint_angles()
        else:
            self.start_config = start_config
        start_pos = self.robot_client.kinematics.FK(self.start_config)[0].getA1().tolist()
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = '/torso_lift_link'
        self.start_pose.pose.position = Point(*start_pos)
        self.start_pose.pose.orientation = Quaternion(0,0,0,1)
        self.record = record
        self.update_rate = rospy.Rate(update_rate)
        self.name = name
        self.goal_pose = None
        self.goal_lock = Lock()

        #Subscribers
        ## Topic for goals to pass directly to controller (no other modules used)
        self.demo_goal_sub = rospy.Subscriber('demo/goal_pose', PoseStamped, self.demo_goal_cb)
        self.controller_goal_sub = rospy.Subscriber('demo/reach_goal', PoseStamped, self.test_reach)
        self.lic_first_goal_sub = rospy.Subscriber('demo/lic1_goal', PoseStamped, self.test_LIC)
        self.planner_goal_sub = rospy.Subscriber('demo/planner_goal', PoseStamped, self.test_planner)

        #Publishers
        ## Publish goal pose to controller
        self.clear_costmap_pub = rospy.Publisher('hrl_planner/refresh_map', Bool)

        #Services
        self.pose_controller_enable_client = rospy.ServiceProxy('pose_controller/enable', EnableHapticMPC)
        self.posture_controller_enable_client = rospy.ServiceProxy('posture_controller/enable', EnableHapticMPC)

        rospy.loginfo("[%s] Checking for Controller Enable/Disable Services" %self.name)
        try:
            self.pose_controller_enable_client.wait_for_service(4)
        except rospy.ROSException as e:
            rospy.logwarn("[%s] Could not find Pose Controller Enable/Disable service. Exiting..." %self.name)
            sys.exit()
        rospy.logwarn("[%s] Found Pose Controller Enable/Disable service." %self.name)
        try:
            self.posture_controller_enable_client.wait_for_service(4)
        except rospy.ROSException as e:
            rospy.logwarn("[%s] Could not find Posture Controller Enable/Disable service. Exiting..." %self.name)
            sys.exit()
        rospy.loginfo("[%s] Found Posture Controller Enable/Disable Service" %self.name)

        #Initialize concurrent threads
        self.trajectory_manager = TrajectoryManager(self.robot_client)
        self.reach_manager = ReachManager(self.robot_client)
        self.state_recorder = StateRecorder(self.robot_client)
        self.plan_requester = TrajectoryRequester()
        self.lic_first = LICFirstRequester()
        self.lic_second = LICSecondRequester()
        self.wait = WaitProcess()

        self.running_processes = [self.wait]

        rospy.sleep(2)
        self.clear_costmap_pub.publish(True)
        rospy.loginfo("[%s] Started." %self.name)
        self.restart_demo()

    def demo_goal_cb(self, ps):
        self.restart_demo()
        with self.goal_lock:
            print "New demo goal activated"
            self.goal_id = get_time_stamp()
            self.goal_pose = ps
            self.goal_status = GoalStatus(self.goal_id, ps)

    def controller_goal_cb(self, ps):
        self.controller_current_goal = ps

    def switch_to_controller(self, controller_name):
        if controller_name == 'pose':
            resp = self.posture_controller_enable_client.call('disable')
            if resp.current_state == 'disabled':
                resp = self.pose_controller_enable_client.call('enable')
                if resp.current_state == 'enabled':
                    rospy.loginfo("[%s] Switched to pose controller." %self.name)
                    return True
                else:
                    rospy.loginfo("[%s] Failed to switch to pose controller." %self.name)
        if controller_name == 'posture':
            resp = self.pose_controller_enable_client.call('disable')
            if resp.current_state == 'disabled':
                resp = self.posture_controller_enable_client.call('enable')
                if resp.current_state == 'enabled':
                    rospy.loginfo("[%s] Switched to posture controller." %self.name)
                    return True
                else:
                    rospy.loginfo("[%s] Failed to switch to posture controller." %self.name)
        return False

    def test_reach(self, goal_pose):
        self.goal_pose = copy.copy(goal_pose)
        self.call_reach(goal_pose)
        if self.monitor_process(self.reach_manager):
            rospy.loginfo("[%s] [TEST CALL] Controller succeeded in reaching goal." %self.name)
        else:
            rospy.loginfo("[%s] [TEST CALL] Controller failed to reach goal." %self.name)

        pullback = 'pose' #'posture' #'pose' #None
        if pullback is None:
            self.restart_demo()
            return
        elif pullback == 'pose':
            self.pose_pullback()
        elif pullback == 'posture':
            self.config_pullback()
        self.restart_demo()

    def call_reach(self, goal_pose):
        if self.switch_to_controller('pose'):
            self.state_recorder.start_recording()
            self.running_processes.append(self.reach_manager)
            self.reach_manager.set_goal_thresh(0.005)
            self.reach_manager.set_goal(goal_pose)
        else:
            rospy.loginfo("[%s] Failed to switch to pose controller. Aborting." %self.name)

    def config_pullback(self):
        self.state_recorder.pause()
        traj = self.state_recorder.get_trajectory_history()
        traj.reverse()
        traj_msg = self.make_traj_msg(traj)
        self.send_trajectory(traj_msg)
        if not self.monitor_process(self.trajectory_manager):
            rospy.loginfo("[%s] Failed to pull arm back from first LIC reach." %self.name)
            self.restart_demo()
            return

    def pose_pullback(self):
        self.state_recorder.pause()
        path = self.state_recorder.get_pose_history()
        path.reverse()
        self.reach_manager.set_goal_thresh(0.05)
        if not self.switch_to_controller('pose'):
            return False
        self.running_processes.append(self.reach_manager)
        for pose in path:
            self.running_processes.append(self.reach_manager)
            self.reach_manager.set_goal(self.make_pose_msg(pose))
            if not self.monitor_process(self.reach_manager):
                return False
        return True

    def test_LIC(self, goal_pose):
        with self.goal_lock:
            self.goal_pose = goal_pose
        self.call_LIC_first(goal_pose)
        if not self.monitor_process(self.lic_first):
            rospy.loginfo("[%s] [LIC TEST] Failed to retrieve first LIC start configuration." %self.name)
            self.restart_demo()
            return
        goal_cfg = self.lic_first.get_config()
        traj_msg = self.interp_trajectory(goal_cfg)
        self.send_trajectory(traj_msg)
        if not self.monitor_process(self.trajectory_manager):
            rospy.loginfo("[%s] [LIC TEST] Failed to reach first lic start configuration." %self.name)
            self.restart_demo()
            return
        self.call_reach(goal_pose)
        succeeded = self.monitor_process(self.reach_manager)
        if succeeded:
            rospy.loginfo("[%s] [LIC TEST] Succeeded in reach from first LIC start config." %self.name)
        self.state_recorder.pause()
        traj = self.state_recorder.get_trajectory_history()
        traj.reverse()
        lic1_start_pos, lic1_start_rot = self.robot_client.kinematics.FK(traj[0])
        lic1_start_pose = self.make_pose_msg(lic1_start_pos.getA1().tolist(), lic1_start_rot)
        lic1_end_pos, lic1_end_rot = self.robot_client.kinematics.FK(traj[-1])
        self.call_LIC_second(goal_pose, lic1_start_pose, lic1_end_pos.getA1().tolist())
        traj_msg = self.make_traj_msg(traj)
        self.send_trajectory(traj_msg)
        if not self.monitor_process(self.trajectory_manager):
            rospy.loginfo("[%s] Failed to pull arm back from first LIC reach." %self.name)
            self.restart_demo()
            return
        if succeeded:
            rospy.loginfo("[%s] Succeeded in pulling arm back from first LIC reach." %self.name)
            self.restart_demo()
            return
        if not self.monitor_process(self.lic_second):
            rospy.loginfo("[%s] [LIC Test] Failed to recieve 2nd LIC start config." %self.name)
            self.restart_demo()
            return
        goal_cfg = self.lic_second.get_config()
        traj_msg = self.interp_trajectory(goal_cfg)
        self.send_trajectory(traj_msg)
        if not self.monitor_process(self.trajectory_manager):
            rospy.loginfo("[%s] [LIC Test] Failed to reach 2nd LIC start config." %self.name)
            self.restart_demo()
            return
        self.call_reach(goal_pose)
        if self.monitor_process(self.reach_manager):
            rospy.loginfo("[%s] [LIC TEST] Succeeded in reach from second LIC start config." %self.name)
        else:
            rospy.loginfo("[%s] [LIC TEST] Failed to reach goal from second LIC start config." %self.name)
        self.state_recorder.pause()
        traj = self.state_recorder.get_trajectory_history()
        traj.reverse()
        traj_msg = self.make_traj_msg(traj)
        self.send_trajectory(traj_msg)
        if not self.monitor_process(self.trajectory_manager):
            rospy.loginfo("[%s] Failed to pull arm back from second LIC reach." %self.name)
            self.restart_demo()
            return
        rospy.loginfo("[%s] Succeeded in pulling arm back from second LIC reach." %self.name)

    def call_LIC_first(self, goal_pose):
        """ Send goal position to LIC.  Returns initial configuration to start from.
        """
        self.running_processes.append(self.lic_first)
        self.lic_first.set_goal(goal_pose)

    def call_LIC_second(self, goal_pose, first_start, first_end):
        """ Send goal position, previous reach start, and previous stop position to LIC.
            Returns initial condition to start reach from.
        """
        self.running_processes.append(self.lic_second)
        self.lic_second.set_goal(goal_pose, first_start, first_end)

    def test_planner(self, goal_pose):
        self.call_planner(goal_pose)
        if not self.monitor_process(self.plan_requester):
            rospy.loginfo("[%s] Failed to retrieve trajectory from planner." %self.name)
            return
        traj = self.plan_requester.get_trajectory()
        rospy.loginfo("[%s] Retrieved trajectory from planner." %self.name)
        self.send_trajectory(traj)
        if self.monitor_process(self.trajectory_manager):
            rospy.loginfo("[%s] Succeeded when using path from planner." %self.name)
        else:
            rospy.loginfo("[%s] Failed when using path from planner." %self.name)

    def call_planner(self, goal_pose):
        """ Send goal pose to planner (only position is used currently).
        """
        self.running_processes.append(self.plan_requester)
        self.plan_requester.set_goal(goal_pose)

    def planner_traj_cb(self, traj):
        """ Thin wrapper to send a trajectory received from the planner on to the controller.
        """
        self.send_trajectory(traj)

    def send_trajectory(self, traj):
        """ Function taking a trajectory and sending sub-goals to the controller based on progress.
        """
        if self.switch_to_controller('posture'):
            self.state_recorder.start_recording()
            self.running_processes.append(self.trajectory_manager)
            self.trajectory_manager.set_trajectory(traj)
        else:
            rospy.loginfo("[%s] Failed to switch to posture controller. Aborting." %self.name)

    def interp_trajectory(self, config):
        current_cfg = self.robot_client.get_joint_angles()
        maxdiff = np.max(np.abs(np.subtract(config, current_cfg)))
        steps = int(np.ceil(maxdiff/np.radians(1)))
        angles = []
        for i in xrange(len(config)):
            angles.append(np.linspace(current_cfg[i],config[i],steps))
        traj_pts = np.array(angles).T
        msg = self.make_traj_msg(traj_pts)
        self.check_joint_limits(msg)
        return msg

    def check_joint_limits(self, traj_msg):
        bad_angles_flag = False
        for point in traj_msg.points:
            if not self.robot_client.kinematics.within_joint_limits(np.array(point.positions)):
                bad_angles_flag = True
            
        if bad_angles_flag:

            rospy.logwarn("[%s] Trajectory Violates Joint Limits." %self.name)
            print "Joint Limits:\r\n"
            print np.degrees(self.robot_client.kinematics.get_joint_limits()[0])
            print np.degrees(self.robot_client.kinematics.get_joint_limits()[1])
        else:
            rospy.loginfo("[%s] Trajectory Within Joint Limits." %self.name)

    def make_traj_msg(self, configs):
        jt = JointTrajectory()
        for cfg in configs:
            jtp = JointTrajectoryPoint()
            jtp.positions = cfg
            jt.points.append(jtp)
        return jt

    def make_pose_msg(self, pos, rot=None):
        if rot is None:
            rot = np.mat(np.eye(3))
        ps = PoseStamped()
        with self.goal_lock:
            ps.header = copy.copy(self.start_pose.header)
        ps.header.stamp = rospy.Time.now()
        ps.pose.position = Point(*pos)
        ps.pose.orientation = Quaternion(*tr.matrix_to_quaternion(rot))
        return ps

    def clear_costmap(self):
        self.clear_costmap_pub.publish(True)

    def cancel_process(self, process):
        if process in self.running_processes:
            process.cancel()
            process.failed = True
            self.running_processes.remove(process)

    def clear_running_processes(self):
        rospy.loginfo("[%s] Clearing all running processes." %(self.name))
        for proc in self.running_processes:
            proc.cancel()
            proc.failed = True
        self.running_processes = []

    def monitor_process(self, process):
        while not rospy.is_shutdown():
            if process.failed:
                rospy.loginfo("[%s] %s Failed" %(self.name, process.name))
                if process in self.running_processes:
                    self.running_processes.remove(process)
                return False
            elif process.succeeded:
                rospy.loginfo("[%s] %s Succeeded" %(self.name, process.name))
                self.running_processes.remove(process)
                return True
            self.update_rate.sleep()

    def restart_demo(self):
        self.clear_costmap()
        if self.alternating_reach(self.start_config):
            rospy.loginfo("[%s][RESTART] Returned to restart configuration." %self.name)
        else:
            rospy.loginfo("[%s][RESTART] Failed to return to restart configuration." %self.name)
            sys.exit()
        with self.goal_lock:
            self.goal_pose = None
            self.goal_status = None
            self.clear_running_processes()
            self.running_processes.append(self.wait)

    def save_goal_status(self, goal_status):
        filename = goal_status.goal_id+'.pkl'
        rospy.loginfo("[%s] Writing trial log %s" %(self.name, filename))
        with open(filename, 'wb') as f:
            pkl.dump(goal_status, f, -1)

    def alternating_reach(self, posture, tries_allowed=3):
        pos, rot =  self.robot_client.kinematics.FK(posture)
        pose_msg = self.make_pose_msg(pos, rot)
        tries = 0
        while tries < tries_allowed:
            self.call_reach(pose_msg)
            self.monitor_process(self.reach_manager)
            traj = self.interp_trajectory(posture)
            self.send_trajectory(traj)
            if self.monitor_process(self.trajectory_manager):
                return True
            tries += 1
        rospy.loginfo("[%s] Alternating reach failed." %self.name)
        return False
            
    def demo_process(self):
        while not rospy.is_shutdown():
            if self.goal_pose is None:
                self.update_rate.sleep()
                continue
            goal_pose = self.goal_pose
            self.run_start = rospy.Time.now()
            self.goal_pose.header.stamp = self.run_start

            #Request LIC 1 Start Configuration (LIC_1_REQ)
            self.call_LIC_first(goal_pose)
            if not self.monitor_process(self.lic_first):
                rospy.loginfo("[%s][Goal %s] Failed to retrieve first LIC start configuration." %(self.name, self.goal_id))
                self.run_duration = rospy.Time.now() - self.run_start
                self.goal_status.times[self.goal_status.LIC_1_REQ] = self.lic_first.get_run_duration().to_sec()
                self.goal_status.update(complete=True, succeeded=False,
                                        final_step=self.goal_status.LIC_1_REQ,
                                        total_time=self.run_duration.to_sec())
                if self.record:
                    self.save_goal_status(self.goal_status)
                self.restart_demo()
                continue
            else:
                self.goal_status.times[self.goal_status.LIC_1_REQ] = self.lic_first.get_run_duration().to_sec()

            #Move to LIC 1 Start Configuration (LIC_1_SETUP)
            goal_cfg = self.lic_first.get_config()
            traj_msg = self.interp_trajectory(goal_cfg)
            self.send_trajectory(traj_msg)
            if not self.monitor_process(self.trajectory_manager):
                rospy.loginfo("[%s][Goal %s] Failed to reach first lic start configuration." %(self.name, self.goal_id))
                self.run_duration = rospy.Time.now() - self.run_start
                self.goal_status.times[self.goal_status.LIC_1_SETUP] = self.trajectory_manager.get_run_duration().to_sec()
                self.goal_status.update(complete=True, succeeded=False,
                                        final_step=self.goal_status.LIC_1_SETUP,
                                        total_time=self.run_duration.to_sec())
                if self.record:
                    self.save_goal_status(self.goal_status)
                self.restart_demo()
                continue
            else:
                self.goal_status.times[self.goal_status.LIC_1_SETUP] = self.trajectory_manager.get_run_duration().to_sec()

            #Reach to goal from LIC 1 Start Configuration (LIC_1_REACH)
            self.call_reach(goal_pose)
            if self.monitor_process(self.reach_manager):
                rospy.loginfo("[%s][Goal %s] Succeeded in reach from first LIC start config." %(self.name, self.goal_id))
                self.run_duration = rospy.Time.now() - self.run_start
                self.goal_status.times[self.goal_status.LIC_1_REACH] = self.reach_manager.get_run_duration().to_sec()
                self.goal_status.update(complete=True, succeeded=True,
                                        final_step=self.goal_status.LIC_1_REACH,
                                        total_time=self.run_duration.to_sec())
                if self.record:
                    self.save_goal_status(self.goal_status)
                self.restart_demo()
                continue
            else:
                self.goal_status.times[self.goal_status.LIC_1_REACH] = self.reach_manager.get_run_duration().to_sec()

            #Pull Back After LIC 1 Reach (LIC_1_RETREAT)
            ## Send second LIC Request
            self.state_recorder.pause()
            traj = self.state_recorder.get_trajectory_history()
            traj.reverse()
            lic1_start_pos, lic1_start_rot = self.robot_client.kinematics.FK(traj[0])
            lic1_start_pose = self.make_pose_msg(lic1_start_pos.getA1().tolist(), lic1_start_rot)
            lic1_end_pos, lic1_end_rot = self.robot_client.kinematics.FK(traj[-1])
            self.call_LIC_second(goal_pose, lic1_start_pose, lic1_end_pos.getA1().tolist())

            pullback_start_time = rospy.Time.now()
            if not self.pose_pullback():
                rospy.loginfo("[%s][Goal %s] Failed to Pull Back from 1st LIC start config." %(self.name, self.goal_id))
                self.run_duration = rospy.Time.now() - self.run_start
                self.goal_status.times[self.goal_status.LIC_1_RETREAT] = (rospy.Time.now() - pullback_start_time).to_sec()
                self.goal_status.update(complete=True, succeeded=False,
                                        final_step=self.goal_status.LIC_1_RETREAT,
                                        total_time=self.run_duration.to_sec())
                if self.record:
                    self.save_goal_status(self.goal_status)
                self.restart_demo()
                continue
            else:
                self.goal_status.times[self.goal_status.LIC_1_RETREAT] = (rospy.Time.now() - pullback_start_time).to_sec()

            #Retrieve LIC 2 Start Configuration (LIC_2_REQ)
            if not self.monitor_process(self.lic_second):
                rospy.loginfo("[%s][Goal %s] Failed to recieve 2nd LIC start config." %(self.name, self.goal_id))
                self.run_duration = rospy.Time.now() - self.run_start
                self.goal_status.times[self.goal_status.LIC_2_REQ] = self.lic_second.get_run_duration().to_sec()
                self.goal_status.update(complete=True, succeeded=False,
                                        final_step=self.goal_status.LIC_2_REQ,
                                        total_time=self.run_duration.to_sec())
                if self.record:
                    self.save_goal_status(self.goal_status)
                self.restart_demo()
                continue
            else:
                self.goal_status.times[self.goal_status.LIC_2_REQ] = self.lic_second.get_run_duration().to_sec()

            #Move to LIC 2 Start Configuration
            goal_cfg = self.lic_second.get_config()
            traj_msg = self.interp_trajectory(goal_cfg)
            self.send_trajectory(traj_msg)
            if not self.monitor_process(self.trajectory_manager):
                rospy.loginfo("[%s][Goal %s] Failed to reach 2nd LIC start config." %(self.name, self.goal_id))
                self.run_duration = rospy.Time.now() - self.run_start
                self.goal_status.times[self.goal_status.LIC_2_SETUP] = self.trajectory_manager.get_run_duration().to_sec()
                self.goal_status.update(complete=True, succeeded=False,
                                        final_step=self.goal_status.LIC_2_SETUP,
                                        total_time=self.run_duration.to_sec())
                if self.record:
                    self.save_goal_status(self.goal_status)
                self.restart_demo()
                continue
            else:
                self.goal_status.times[self.goal_status.LIC_2_SETUP] = self.trajectory_manager.get_run_duration().to_sec()

            #Reach from LIC 2 Start Configuration
            self.call_reach(goal_pose)
            if self.monitor_process(self.reach_manager):
                rospy.loginfo("[%s][Goal %s] Succeeded in reach from second LIC start config." %(self.name, self.goal_id))
                self.run_duration = rospy.Time.now() - self.run_start
                self.goal_status.times[self.goal_status.LIC_2_REACH] = self.reach_manager.get_run_duration().to_sec()
                self.goal_status.update(complete=True, succeeded=True,
                                        final_step=self.goal_status.LIC_2_REACH,
                                        total_time=self.run_duration.to_sec())
                if self.record:
                    self.save_goal_status(self.goal_status)
                self.restart_demo()
                continue
            else:
                self.goal_status.times[self.goal_status.LIC_2_REACH] = self.reach_manager.get_run_duration().to_sec()
                current_cfg = self.robot_client.get_joint_angles()
                traj_msg = self.make_traj_msg([current_cfg])
                self.send_trajectory(traj_msg)

            #Call Planner Repeatedly...
            try_num = 0
            pulled_back = False
            self.goal_status.planner = {}
            while not rospy.is_shutdown():
                self.goal_status.planner[try_num] = {}

                #Request a plan from the planner (PLAN_REQUEST)
                self.call_planner(self.goal_pose)
                if not self.monitor_process(self.plan_requester):
                    rospy.loginfo("[%s][Goal %s] Failed to retrieve trajectory from planner." %(self.name, self.goal_id))
                    self.goal_status.planner[try_num][self.goal_status.PLAN_REQUEST] = self.plan_requester.get_run_duration().to_sec()
                    if pulled_back:
                        rospy.loginfo("[%s][Goal %s] Already pulled back and cannot plan. Exiting." %(self.name, self.goal_id))
                        self.run_duration = rospy.Time.now() - self.run_start
                        self.goal_status.update(complete=True, succeeded=False,
                                                final_step=self.goal_status.PLAN_REQUEST,
                                                total_time=self.run_duration.to_sec())
                        if self.record:
                            self.save_goal_status(self.goal_status)
                        self.restart_demo()
                        break

                    pullback_start = rospy.Time.now()
                    if not self.alternating_reach(goal_cfg): #RETURN TO LIC 2 SETUP
                        rospy.loginfo("[%s][Goal %s] Failed to pull back to replan." %(self.name, self.goal_id))
                        self.goal_status.planner[try_num][self.goal_status.PLAN_PULLBACK] = (rospy.Time.now()-pullback_start).to_sec()
                        self.run_duration = rospy.Time.now() - self.run_start
                        self.goal_status.update(complete=True, succeeded=False,
                                                final_step=self.goal_status.PLAN_PULLBACK,
                                                total_time=self.run_duration.to_sec())
                        if self.record:
                            self.save_goal_status(self.goal_status)
                        self.restart_demo()
                        break
                    else:
                        self.goal_status.planner[try_num][self.goal_status.PLAN_PULLBACK] = (rospy.Time.now()-pullback_start).to_sec()
                        rospy.loginfo("[%s][Goal %s] Pulled back to replan." %(self.name, self.goal_id))
                        pulled_back = True
                        try_num += 1
                        continue
                else:
                    self.goal_status.planner[try_num][self.goal_status.PLAN_REQUEST] = self.plan_requester.get_run_duration().to_sec()

                #Execute the plan from the planner (PLAN_REACH)
                pulled_back = False
                traj = self.plan_requester.get_trajectory()
                self.send_trajectory(traj)
                if self.monitor_process(self.trajectory_manager):
                    rospy.loginfo("[%s][Goal %s] Succeeded using path from planner." %(self.name, self.goal_id))
                    self.run_duration = rospy.Time.now() - self.run_start
                    self.goal_status.planner[try_num][self.goal_status.PLAN_REACH] = self.trajectory_manager.get_run_duration().to_sec()
                    self.goal_status.update(complete=True, succeeded=True,
                                            final_step=self.goal_status.PLAN_REACH,
                                            total_time=self.run_duration.to_sec())
                    if self.record:
                        self.save_goal_status(self.goal_status)
                    self.restart_demo()
                    break
                else:
                    rospy.loginfo("[%s][Goal %s] Failed to reach goal using path from planner." %(self.name, self.goal_id))
                    self.goal_status.planner[try_num][self.goal_status.PLAN_REACH] = self.trajectory_manager.get_run_duration().to_sec()

                #Reach from planner end-point (POST_PLAN_REACH)
                self.call_reach(goal_pose)
                if self.monitor_process(self.reach_manager):
                    rospy.loginfo("[%s][Goal %s] Succeeded in reach from failed plan endpoint." %(self.name, self.goal_id))
                    self.run_duration = rospy.Time.now() - self.run_start
                    self.goal_status.planner[try_num][self.goal_status.POST_PLAN_REACH] = self.reach_manager.get_run_duration().to_sec()
                    self.goal_status.update(complete=True, succeeded=True,
                                            final_step=self.goal_status.POST_PLAN_REACH,
                                            total_time=self.run_duration.to_sec())
                    if self.record:
                        self.save_goal_status(self.goal_status)
                    self.restart_demo()
                    break
                else:
                    self.goal_status.planner[try_num][self.goal_status.POST_PLAN_REACH] = self.reach_manager.get_run_duration().to_sec()
                    current_cfg = self.robot_client.get_joint_angles()
                    traj_msg = self.make_traj_msg([current_cfg])
                    self.send_trajectory(traj_msg)
                    try_num += 1


if __name__=='__main__':
    import argparse
    p = argparse.ArgumentParser(description="ROS Node for managing interaction of portions of HRL DarpaM3 Final Demo",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    valid_robots= ["darci", "darci_sim"]
    p.add_argument('robot', action='store', type=str, choices=valid_robots,
                    help = 'Specify which robot is being used.')
    p.add_argument('--arm', '-a', action='store', dest='arm', default='l',
                    type=str, help='Specify which arm to use (l or r)')
    p.add_argument('--test','-t', action='store_true', dest='test', default=False,
                   help="Select 'test' to not automatically transition between components")
    p.add_argument('--record','-r', action='store_true', dest='record', default=False,
                   help="Indicate whether to save data on goal attempts.")
    args = p.parse_args()

    rospy.init_node('robot_manager')
    if args.robot == 'darci':
        import darci_client
        robot_client = darci_client.DarciClient()
    elif args.robot == 'darci_sim':
        import darci_sim_client
        robot_client = darci_sim_client.DarciSimClient()

    # original forward facing on left side
    start_config= [-0.5372, -0.23174, -0.130, 1.824, 0.158, 0.3924, 0.1908]
    # relaxed forward facing on left side
    #start_config = np.radians([-28., -10., 10., 83., -138., -23., 23.])
    #start_config = np.radians([-27.30, -33.81,3.0,90.54, -70.41, -22.,23])
   

    #start_config= [-0.20, -0.31, 0.83, 1.66, -1.7, -0.40, 0.2]
#    start_config = np.radians([-18.45, -10.99, 38.53, 91.53, -57.83, -4.40,  37.69])

    #start_config= np.radians([-30.0, -25.0, 15.0, 98.0, 2.0, 25.0, -4.0])

    manager = RobotManager(robot_client, start_config=start_config, record=args.record)
    if args.test:
        rospy.spin()
    else:
        manager.demo_process()
