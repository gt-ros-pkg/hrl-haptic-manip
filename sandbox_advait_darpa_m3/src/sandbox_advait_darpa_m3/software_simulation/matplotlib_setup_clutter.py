
import matplotlib.pyplot as pp
import numpy as np, math
import copy

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy
import hrl_lib.matplotlib_util as mpu
import hrl_lib.util as ut

import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa
import hrl_common_code_darpa_m3.robot_config.three_link_planar_cuboid as d_robot

import hrl_common_code_darpa_m3.software_simulation_setup.obstacles as sso

from geometry_msgs.msg import PointStamped


class ClutterSetup():
    def __init__(self, show_env=True, mpl_events=True):
        self.click_action = ('', False)
        self.fig = pp.figure()
        #self.ax = pp.subplot(111)
        self.ax = pp.gca()
        self.ax.axis('equal')
        pp.hold(True)

        if mpl_events:
            self.fig.canvas.mpl_connect('button_press_event', self.on_click)
            self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.fixed_obstacles_list = []
        self.moveable_obstacles_list = []
        self.goal = [0.5, 0., 0.]

        self.arm_kinematics = gsa.RobotSimulatorKDL(d_robot)
        self.show_env = show_env

        self.fig.canvas.draw()
    
    def find_closest_obstacle_index(self, x, y):
        mat = np.matrix(self.fixed_obstacles_list).T[0:2,:]
        d_arr = ut.norm(mat-np.matrix([x,y]).T).A1
        idx = np.argmin(d_arr)
        return idx
    
    #-------- matplotlib callbacks -----------
    def on_key(self, event, action=''):
        k = event.key
        if k == 'l':
            self.save_reach_problem_dict('reach_problem_dict.pkl')
            print 'Logged the reach problem dict.'
            return
        if k == 'g':
            action = 'set_goal'
            self.add_goal(self.goal[0], self.goal[1])
        if k == 'e':
            action = 'set_obstacles'

        if self.click_action[0] == action:
            self.click_action = (action, not self.click_action[1])
        else:
            self.click_action = (action, True)

        print 'click_action:', self.click_action

    def on_click(self, event):
        x, y = event.ydata, -event.xdata
        x, y = float(x), float(y)
        
        if self.click_action[1] == False:
            return
        ac = self.click_action[0]

        if ac == 'set_obstacles':
            if event.button == 1:
                self.add_fixed_obstacle(x, y, 0.01)
            elif event.button == 3:
                self.remove_fixed_obstacle(x, y)
        elif ac == 'set_goal':
            self.add_goal(x, y)

        self.redraw()

    #------- drawing functions ----------------
    def draw_fixed_obstacle(self, cx, cy, radius):
        mpu.plot_circle(-cy, cx, radius, 0., math.pi*2, color='r')

    # in the future, make grid limits depend on the kinematics of the
    # arm.
    def draw_grid(self):
        x_linspace = np.linspace(0, 1., 25) # every four cm.
        y_linspace = np.linspace(-0.8, 0.8, 40) # every four cm.
        x,y = np.meshgrid(x_linspace, y_linspace)
        pp.plot(-y, x, 'k.', alpha=1., ms=1)

    def draw_obstacles(self):
        if len(self.fixed_obstacles_list) > 0:
            arr = np.array(self.fixed_obstacles_list)
            pos_arr = arr[:,0:2]
            rad_arr = arr[:,2].flatten()
            for i in range(len(rad_arr)):
                cx, cy = pos_arr[i,0], pos_arr[i,1]
                mpu.plot_circle(-cy, cx, rad_arr[i], 0., math.pi*2, color='r')

    def draw_manip_configs(self):
        n_jts = self.arm_kinematics.n_jts
        self.arm_kinematics.plot_arm([0]*n_jts, 'b', alpha=0.1, flip_xy=True)

    def draw_goal(self):
        pp.plot([-self.goal[1]], [self.goal[0]], 'gx', ms=8, mew=2)

    def redraw_list(self):
        self.draw_grid()
        if self.show_env:
            self.draw_obstacles()
            self.draw_manip_configs()
        self.draw_goal()

    def redraw(self):
        self.ax.clear()
        self.redraw_list()
        pp.draw()

    #--------- creating the reaching problem instance ------
    def remove_fixed_obstacle(self, x, y):
        idx = self.find_closest_obstacle_index(x, y)
        obs = np.matrix(self.fixed_obstacles_list[idx]).T[0:2]
        d = np.linalg.norm(obs-np.matrix([x,y]).T)
        if d < 0.03:
            self.fixed_obstacles_list.pop(idx)

    def add_goal(self, x, y):
        self.goal[0] = x
        self.goal[1] = y

    def add_fixed_obstacle(self, x, y, radius):
        self.fixed_obstacles_list.append((x,y,radius))

    def save_reach_problem_dict(self, fname):
        d = {}

        total_num = 20
        z = np.ones(total_num) * 0.0
        x_move, y_move = sso.generate_far_locations(-100, total_num)
        moveable_position = np.row_stack((x_move,y_move,z)).T.tolist()

        x_fix, y_fix = sso.generate_far_locations(100, total_num)
        if len(self.fixed_obstacles_list) > 0:
            #            arr = np.array(self.fixed_obstacles_list, dtype=float)
            x_l, y_l, r_l = zip(*self.fixed_obstacles_list)

            for i in range(len(self.fixed_obstacles_list)):
                x_fix[i] = x_l[i]
                y_fix[i] = y_l[i]
            r = r_l[0]
        else:
            r = 0.2
        fixed_position = np.row_stack((x_fix, y_fix, z)).T.tolist()

        d['fixed_dimen'] = [[r, r, 0.2] for i in range(total_num)]
        d['fixed_position'] = fixed_position
        d['num_fixed_used'] = len(self.fixed_obstacles_list)
        d['num_move_used'] = 0
        d['num_total'] = total_num
        d['moveable_dimen'] = [[r, r, 0.2] for i in range(total_num)]
        d['moveable_position'] = moveable_position
        d['goal'] = self.goal
        
        ut.save_pickle(d, fname)

    def init_from_dict(self, d):
        n_fixed = d['num_fixed_used']
        fixed_pos = np.array(d['fixed_position'])
        rad = d['fixed_dimen'][0][0]
        self.fixed_obstacles_list = np.row_stack((fixed_pos[0:n_fixed,0:2].T, np.ones(n_fixed)*rad)).T.tolist()
        self.goal = d['goal']


if __name__ == '__main__':
    import optparse

    p = optparse.OptionParser()
    p.add_option('--pkl', action='store', dest='pkl', type='string',
                 help='pkl to load and plot (so that it can be modified).')
    opt, args = p.parse_args()

    cs = ClutterSetup()

    if opt.pkl:
        d = ut.load_pickle(opt.pkl)
        cs.init_from_dict(d)

    cs.redraw()
    pp.show()




