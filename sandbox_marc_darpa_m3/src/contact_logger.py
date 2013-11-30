#!/usr/bin/python 
import numpy as np
from threading import RLock
import sys
import roslib
roslib.load_manifest('sandbox_marc_darpa_m3')
import rospy
from collections import deque
from hrl_haptic_manipulation_in_clutter_msgs.msg import BodyDraw
from hrl_haptic_manipulation_in_clutter_msgs.msg import SkinContact
import cPickle
from std_msgs.msg import Bool, Empty

from hrl_haptic_manipulation_in_clutter_srvs.srv import StartLog, StartLogResponse
from hrl_msgs.msg import FloatArrayBare
import sys


node_name = "simulation_logger"

class ODELogClient():
    def __init__(self):
        rospy.loginfo('Waiting for service')
        rospy.wait_for_service('start_contact_logger')
        rospy.loginfo('done.')
        self.start_srv_proxy = rospy.ServiceProxy('start_contact_logger', StartLog)
        self.end_srv_proxy = rospy.ServiceProxy('stop_contact_logger', StartLog)

    def start_logging(self):
        self.start_srv_proxy()

    def end_logging(self):
        self.end_srv_proxy()


class ODEContactLogger():
    def __init__(self, pkl_name, robot):
        self.pkl_name = pkl_name
        self.forces = deque()
        self.force_locs = deque()
        self.force_nrmls = deque()
        self.link_names = deque()
        self.pts = deque()
        self.obstacle_positions = deque()
        self.dist_to_goal = deque()
        self.f_time = deque()
        self.pos_time = deque()
        self.joint_time_stamp = deque()
        self.ee_position = deque()
        self.arm_angles = deque()
        self.arm_angle_rates = deque()
        self.ee_pos_filter_buf = deque()
        self.ee_pos_filtered_hist = deque()
        self.lock = RLock()

        self.stop_pub = rospy.Publisher('/epc/stop', Bool)

        self.goal = np.array(rospy.get_param('m3/software_testbed/goal')).reshape(3,1)
        self.num_fixed = rospy.get_param('m3/software_testbed/num_fixed')
        self.num_movable = rospy.get_param('m3/software_testbed/num_movable')
        self.num_tot = rospy.get_param('m3/software_testbed/num_total')
        self.right_arm = 0

        self.start_serv = rospy.Service('start_contact_logger', StartLog, self.run)
        self.stop_serv = rospy.Service('stop_contact_logger', StartLog, self.end_logging)

        self.robot = robot
        self.run_now = False
        #rospy.Subscriber("/sim_arm/body_visualization", BodyDraw, self.viz_callback)
        #this is to subscribe to the raw forces seen by the world, not the taxel force measurements
        rospy.Subscriber("/skin/contacts_unused", SkinContact, self.skin_callback)
        rospy.Subscriber('/sim_arm/joint_angles', FloatArrayBare, self.joint_callback)

    def write_to_pkl(self, file_name):
        f_mag_buf = []
        for i in xrange(len(self.link_names)):
            for j in xrange(len(self.forces[i])):
                f_mag_buf.append(np.linalg.norm(np.array(self.forces[i][j])))
                if np.linalg.norm(np.array(self.forces[i][j]))>100:
                    print "this is max :", self.forces[i][j]
        if f_mag_buf != []:
            try:
                f_max = np.max(np.array(f_mag_buf))    
                f_avg = np.mean(np.array(f_mag_buf))
                f_std = np.std(np.array(f_mag_buf))
                f_med = np.median(np.array(f_mag_buf))
            except:
                f_max = 0
                f_avg = 0
                f_std = 0
                f_med = 0
        else:
            f_max = 0
            f_avg = 0
            f_std = 0
            f_med = 0

        print "stats on forces", f_max, f_avg, f_std
        data = {'total_approx_time': self.joint_time_stamp[-1]-self.joint_time_stamp[0],
                'max_force': f_max, 
                'avg_force': f_avg, 
                'std_force': f_std, 
                'median_force': f_med,
                'forces':self.forces, 
                'skin_locations': self.force_locs,
                'skin_nrmls': self.force_nrmls,
                'f_time':self.f_time, 
                'd_to_goal':self.dist_to_goal,
                'ee_position':self.ee_position,
                'dist_and_angles_time':self.joint_time_stamp, 
                'arm_angles':self.arm_angles,
                'arm_angle_rates':self.arm_angle_rates, 
                'contact_points_x_y_z':self.pts, 
                'link_names':self.link_names}
          
        # 'obst_start_state': self.obstacle_positions[0], 
        # 'obst_end_state': self.obstacle_positions[-1],

        # data = {'forces': self.forces, 'f_time': self.f_time,
        #         'obst_positions': self.obstacle_positions, 'pos_time': self.pos_time,
        #         'd_to_goal': self.dist_to_goal, 'd_to_goal_time': self.joint_time_stamp,
        #         'arm_angles': self.arm_angles}
        output = open(file_name, 'w')
        cPickle.dump(data, output)
        output.close()

    def skin_callback(self, msg):
        self.lock.acquire()
        if self.run_now == True:
            force_buf = []
            force_locs_buf = []
            force_nrmls_buf = []
            link_names_buf = []
            pts_buf_x = []
            pts_buf_y = []
            pts_buf_z = []
            self.f_time.append(msg.header.stamp.to_sec())
            for i in xrange(len(msg.link_names)):
                force_buf.append(np.array([msg.forces[i].x, msg.forces[i].y, msg.forces[i].z]))
                force_locs_buf.append(
                    np.array([msg.locations[i].x, 
                              msg.locations[i].y, 
                              msg.locations[i].z]))
                force_nrmls_buf.append(
                    np.array([msg.normals[i].x, 
                              msg.normals[i].y, 
                              msg.normals[i].z]))
                link_names_buf.append(msg.link_names[i])
                pts_buf_x.append(np.array(msg.pts_x[i].data))
                pts_buf_y.append(np.array(msg.pts_y[i].data))
                pts_buf_z.append(np.array(msg.pts_z[i].data))
            self.forces.append(force_buf)
            self.force_locs.append(force_locs_buf)
            self.force_nrmls.append(force_nrmls_buf)
            self.link_names.append(link_names_buf)
            pts_ar =[pts_buf_x, pts_buf_y, pts_buf_z]
            self.pts.append(pts_ar)
            
            #self.    = msg.
        self.lock.release()

    # def viz_callback(self, msg):
    #     self.lock.acquire()
    #     if self.run_now == True:
    #         self.pos_time.append(msg.header.stamp.to_sec())
    #         self.obstacle_positions.append(np.array([x.data for x in msg.obst_loc]))

    #         #could change this to have separate call back for angles
    #         #if we want more accurate time stamp for them as well
    #         # q = self.ode_arm.get_joint_angles(self.right_arm)
    #         # self.arm_angles.append(q)
    #         # cur_pos, _  = self.ode_arm.arms.FK_all(self.right_arm, q) 
    #         # self.dist_to_goal.append(self.goal-cur_pos.reshape(3,1))
    #         # self.joint_time_stamp.append(rospy.get_time())
    #     self.lock.release()

    def joint_callback(self, msg):
        self.lock.acquire()
        if self.run_now == True:

            self.arm_angles.append(msg.data)
            self.arm_angle_rates.append(self.robot.get_joint_velocities())
            cur_pos, _  = self.robot.kinematics.FK(msg.data) 
            self.ee_position.append(cur_pos)
            self.dist_to_goal.append(self.goal-cur_pos.reshape(3,1))
            self.joint_time_stamp.append(rospy.get_time())

            # THIS IS HACKY FOR NOW better methods might be to
                # - look at window filter size and do a better job filtering motion from Cody
                # - look at filtered derivative and fit a threshold so that we are looking at velocity
            self.ee_pos_filter_buf.append(cur_pos)
            if len(self.ee_pos_filter_buf) > 50:
                self.ee_pos_filter_buf.popleft()
                self.ee_pos_filtered_hist.append(np.mean(self.ee_pos_filter_buf, 0))
            if len(self.ee_pos_filtered_hist) > 1000:
                self.ee_pos_filtered_hist.popleft()
                if np.linalg.norm(self.ee_pos_filtered_hist[-1]-self.ee_pos_filtered_hist[0]) < 0.001:
                    self.stop_pub.publish(Bool(True))


        self.lock.release()

    def end_logging(self, msg):
        self.lock.acquire()
        self.run_now = False
        rospy.loginfo("writing to file now")
        self.write_to_pkl(self.pkl_name)
        #self.write_to_pkl(msg.file_name)
        rospy.signal_shutdown('Done Logging')
        self.lock.release()
        return StartLogResponse()

    def run(self, msg): 
        rospy.loginfo('Begin logging')
        self.run_now = True
        return StartLogResponse()

if __name__ == '__main__':


    import optparse
    p = optparse.OptionParser()

    p.add_option('--fname', action='store', dest='fname', type='string',
                 help='name of pkl file with logger result.')
    p.add_option('--cody', action='store_true', dest='cody',
                 help='task monitoring for cody')
    p.add_option('--sim', action='store_true', dest='sim',
                 help='software simulation')
    p.add_option('--planar_three_link_capsule', action='store_true', 
                 dest='three_link_planar', default=False,
                 help='this is to the use the three_link_planar robot config file')
    p.add_option('--six_link_planar', action='store_true', 
                 dest='six_link_planar', default=False,
                 help='this is to the use the six_link_planar robot config file')
    p.add_option('--multi_link_three_planar', action='store_true', 
                 dest='multi_link_three', default=False,
                 help='this is to the use the multi_link_three_planar robot config file')
    p.add_option('--multi_link_four_planar', action='store_true', 
                 dest='multi_link_four', default=False,
                 help='this is to the use the multi_link_four_planar robot config file')
    p.add_option('--multi_link_five_planar', action='store_true', 
                 dest='multi_link_five', default=False,
                 help='this is to the use the multi_link_five_planar robot config file')
    p.add_option('--multi_link_six_planar', action='store_true', 
                 dest='multi_link_six', default=False,
                 help='this is to the use the multi_link_six_planar robot config file')
    p.add_option('--multi_link_seven_planar', action='store_true', 
                 dest='multi_link_seven', default=False,
                 help='this is to the use the multi_link_seven_planar robot config file')
    p.add_option('--multi_link_eight_planar', action='store_true', 
                 dest='multi_link_eight', default=False,
                 help='this is to the use the multi_link_eight_planar robot config file')

    opt, args = p.parse_args()

    if opt.cody:
        rospy.logerr("I don't have code yet to log Cody's forces, need to fix ... exiting")
        sys.exit()

#         import hrl_cody_arms.cody_arm_client as cac
#         #import sandbox_advait_darpa_m3.cody.cody_guarded_move as cgm
#         from sandbox_advait_darpa_m3.cody.cody_guarded_move import Cody_SkinClient
#         #from sandbox_advait_darpa_m3.cody.cody_guarded_move import MobileSkinEPC_Cody


#         if opt.arm == None:
#             rospy.logerr('Need to specify --arm_to_use.\nExiting...')
#             sys.exit()

#         robot = cac.CodyArmClient(opt.arm)

#         if opt.arm == 'r':
#             max_lim = np.radians([ 120.00, 122.15, 0., 144., 122.,  45.,  45.])
#             min_lim = np.radians([ -47.61,  -20., -77.5,   0., -80., -45., -45.])
#         elif opt.arm == 'l':
#             max_lim = np.radians([ 120.00,   20.,  77.5, 144.,   80.,  45.,  45.])
#             min_lim = np.radians([ -47.61, -122.15, 0.,   0., -122., -45., -45.])
#         else:
#             rospy.logerr('Unknown arm.\nExiting...')
#             sys.exit()

#         robot.kinematics.joint_lim_dict['max'] = max_lim
#         robot.kinematics.joint_lim_dict['min'] = min_lim

#         #robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.16]).T) # tube
#         #robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.01]).T) # stub
#         robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.04]).T) # stub with mini45

#         #scl = cgm.Cody_SkinClient(skin_topic_list)
#         scl = Cody_SkinClient(skin_topic_list)

# #        epcon = cgm.MobileSkinEPC_Cody(robot, scl)
# #        epcon = MobileSkinEPC_Cody(robot, scl)
#         epc = Skin_EPC_marc(robot, scl)
#         while robot.get_ep() == None:
#             rospy.sleep(0.1)
#         jep_start = robot.get_ep()

#         q_home = np.matrix(jep_start)

#         testbed = 'hardware_in_loop_cody'

#         jep = robot.get_joint_angles()
#         cur_pos, _ = robot.kinematics.FK(jep)
#         goal_pos = np.matrix(cur_pos)

#         # cur_pos, _ = robot.kinematics.FK(jep)
#         # goal_pos = np.matrix(cur_pos)

    if opt.sim:
        if opt.three_link_planar == True:
            import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
            #import hrl_common_code_darpa_m3.robot_config.multi_link_three_planar as d_robot
        elif opt.six_link_planar == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_six_planar as d_robot
        elif opt.multi_link_three == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_three_planar as d_robot
        elif opt.multi_link_four == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_four_planar as d_robot
        elif opt.multi_link_five == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_five_planar as d_robot
        elif opt.multi_link_six == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_six_planar as d_robot
        elif opt.multi_link_seven == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_seven_planar as d_robot
        elif opt.multi_link_eight == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_eight_planar as d_robot
        else:
            rospy.logerr("You didn't give a robot config option for logging, see --help \n Exiting ...")
            sys.exit()
        import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa

        robot = gsa.ODESimArm(d_robot)

        goal_pos = np.matrix(rospy.get_param('m3/software_testbed/goal')).T

    rospy.init_node("simulation_contact_logging")
    rospy.loginfo("starting logging server")
    logger = ODEContactLogger(opt.fname, robot)
    rospy.loginfo("got past object instantiation")
    rospy.spin()

    # sample code for what should go into a controller that wants to
    # use this logger.
    # if False:
    #     rospy.init_node('test_logging')
    #     log_client = cl.ODELogClient()
    #     rospy.sleep(0.5)
    #     log_client.start_logging()
    #     rospy.sleep(10.)
    #     log_client.end_logging()

