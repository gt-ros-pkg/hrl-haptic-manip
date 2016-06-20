#/usr/bin/env python

import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sttr_msgs.msg import WrenchArray
from hrl_msgs.msg import FloatArray
import numpy as np
import math

### Communication Module ###


class SPR2YZTSLegliftComNode(object):
    def __init__(self):
        # Initialize status variables
        self.robot_ja = [0.,0.,0.,0.,0.,0.]
        self.robot_desired_ja = [0.,0.,0.,0.,0.,0.]
        self.joint_error = [0.,0.,0.,0.,0.,0.]
        n = 20 # size of moving average for force/torque arrays
        self.force_y_array_1 = [0] * n
        self.force_z_array_1 = [0] * n
        self.torque_y_array_1 = [0] * n
        self.force_y_array_2 = [0] * n
        self.force_z_array_2 = [0] * n
        self.torque_y_array_2 = [0] * n
        self.max_velocity = 100 # maximum velocity that the low level controller in m/s
        self.time_from_start = 0.1 # s

        # Subscribers
        rospy.Subscriber('/simple_prismatic_robot_controller_2/state',JointTrajectoryControllerState,self.update_robot_kinematic_state)
        rospy.Subscriber('/gazebo/sprjt',WrenchArray,self.update_ft)
        rospy.Subscriber('/gazebo/leg_angles',FloatArray,self.update_actual_theta)

        # Publishers
        move_pub = rospy.Publisher('/simple_prismatic_robot_controller_2/command',JointTrajectory)

    def update_robot_kinematic_state(self, msg):
        self.robot_ja = [msg.actual.positions[0], msg.actual.positions[1], msg.actual.positions[2],msg.actual.positions[3], msg.actual.positions[4], msg.actual.positions[5]]
        self.robot_desired_ja = [msg.desired.positions[0], msg.desired.positions[1], msg.desired.positions[2],msg.desired.positions[3], msg.desired.positions[4], msg.desired.positions[5]]
        self.joint_error = (np.array(self.robot_desired_ja)-np.array(self.robot_ja)).tolist()

    def update_ft(self,msg):
        self.force_y_array_1.append(msg.force_y[1]) # using the 1st and 4th indicies from the message because these are from the second joint on the SPRs rather than the rotating frame
        self.force_z_array_1.append(msg.force_z[1])
        self.torque_y_array_1.append(msg.torque_y[1])
        self.force_y_array_2.append(msg.force_y[4])
        self.force_z_array_2.append(msg.force_z[4])
        self.torque_y_array_2.append(msg.torque_y[4])
        del self.force_y_array_1[0]
        del self.force_z_array_1[0]
        del self.torque_y_array_1[0]
        del self.force_y_array_2[0]
        del self.force_z_array_2[0]
        del self.torque_y_array_2[0]
        self.force_y_1 = sum(self.force_y_array_1)/len(self.force_y_array_1)
        self.force_z_1 = sum(self.force_z_array_1)/len(self.force_z_array_1)
        self.torque_y_1 = sum(self.torque_y_array_1)/len(self.torque_y_array_1)
        self.force_y_2 = sum(self.force_y_array_2)/len(self.force_y_array_2)
        self.force_z_2 = sum(self.force_z_array_2)/len(self.force_z_array_2)
        self.torque_y_2 = sum(self.torque_y_array_2)/len(self.torque_y_array_2)

    def initialize_msg(self):
        self.msg = JointTrajectory()
        self.msg.header.frame_id = "/base_link"
        jp = JointTrajectoryPoint()
        jp.positions = [0.,0.,0.,0.,0.,0.]
        jp.time_from_start = rospy.Duration(0.)
        self.msg.points.append(jp)
        self.msg.joint_names = rospy.get_param('/simple_prismatic_robot_controller_2/joints')

    def create_msg(self,goal,time):
        self.msg.points[0].positions = goal
        self.msg.points[0].time_from_start = rospy.Duration(time)
        self.msg.header.stamp = rospy.Time.now()

    def send_goal(self,goal,time,freq,previous_goal=None):
        jerror_threshold = 50 # arbitrarily chosen for now
        r = rospy.Rate(freq)
        goal_trajectories = self.linear_interpolation_goal(goal,time,freq,previous_goal)
        for i in range(len(goal_trajectories)):
            self.create_msg(goal_trajectories[i],1./freq)
            self.move.publish(self.msg)
            r.sleep()
            jerror_cond = np.linalg.norm(np.array(self.joint_error)) > jerror_threshold
            while jerror_cond:
                self.move.publish(self.msg)
                r.sleep()
                print 'wait for joint error small than threshold...', 'err=',  np.linalg.norm(np.array(self.joint_error(goal_trajectories[i])))
                jerror_cond = np.linalg.norm(np.array(self.joint_error(goal_trajectories[i]))) > jerror_threshold
        return True

    def linear_interpolation_goal(self,goal,time,freq,previous_goal=None):
        if previous_goal == None:
            init = self.robot_desired_ja
        else:
            init = previous_goal
        goal_trajectory = []
        increment = (np.array(goal)-np.array(init))/(time*freq)
        for i in range(int(time*freq)):
            next_goal = init+(i+1)*increment
            goal_trajectory.append(next_goal.tolist())
        return goal_trajectory

    def lift(self,goal):
        self.initialize_msg()
        self.goal = goal
        # we are sending desired joint angles, not actual joint angles because this is an impedance controller and we don't know if the joint will actually reach the goal
        # determine the required velocity based on the maximum required velocity out of all the joints
        required_velocity = max(abs(np.array(self.goal)-np.array(self.robot_desired_ja))/self.time_from_start)
        if required_velocity < self.max_velocity:
            self.send_goal(self.goal,self.time_from_start,1000)
        else:
            print "new time from start: ",(required_velocity*self.time_from_start)/self.max_velocity
            self.send_goal(self.goal,(required_velocity*self.time_from_start)/self.max_velocity,1000)

        print "end time: ",rospy.get_time()


if __name__ == '__main__':
    rospy.init_node('spr_lift')
    spr_lift = sprLift()
    rospy.sleep(5.)
    #spr_lift.lift([0.25])
    spr_lift.lift([0.,0.,0.15,0.,0.,0.15])
    #spr_lift.lift([-0.24,0.,0.2,-0.24,0.,0.3])
    rospy.sleep(10.)
