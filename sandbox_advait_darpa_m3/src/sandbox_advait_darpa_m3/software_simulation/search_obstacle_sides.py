
import numpy as np, math
import shapely.geometry as sg
import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import hrl_lib.geometry as hg


class ObstacleSideAnalyzer():
    def __init__(self, kdl_arm):
        self.kdl_arm = kdl_arm
        self.previous_config_list = []
        self.attempted_config_obstacle_combo_list = []

    # convert robot arm configuration and goal location into a polygon
    # that can be used to perform interior/exterior checks for all the
    # obstacles.
    def polygon_from_config(self, q):
        coords = [p[0:2] for p in self.kdl_arm.arm_config_to_points_list(q)]
        max_x = 2.0
        max_y = 2.0
        coords.append((max_x, p[1]))
        coords.append((max_x, max_y))
        coords.append((0, max_y))
        po = sg.Polygon(coords)
        return po

    # obs_coord_list - list of array-like of len 2 or 3
    def assign_side_to_obstacles(self, q, obs_coord_list):
        po = self.polygon_from_config(q)
        obs_side_list = []
        for obs in obs_coord_list:
            pt = sg.Point((obs[0], obs[1]))
            obs_side_list.append(po.contains(pt))
        return obs_side_list

    # save this configuration as one that the robot has been in.
    def save_robot_config(self, q):
        self.previous_config_list.append(q)

    # is the obs_side_list a state that the robot has been in before.
    def is_prev_done_state(self, obs_side_list, obs_coord_list):
        prev_done = False
        for q in self.previous_config_list:
            sl = self.assign_side_to_obstacles(q, obs_coord_list)
            if sl == obs_side_list:
                prev_done = True
                break
        return prev_done

    def is_prev_attempted_switch(self, q, o, obs_l):
        side_l = self.assign_side_to_obstacles(q, obs_l)
        for q2, o2 in self.attempted_config_obstacle_combo_list:
            side_l2 = self.assign_side_to_obstacles(q2, obs_l)
            if side_l == side_l2 and np.linalg.norm(o-o2) < 0.02:
                return True
        return False

    def add_switch_attempt(self, q, obs):
        self.attempted_config_obstacle_combo_list.append((q, obs))

    # obs - 2x1 or 3x1 np matrix
    def generate_waypoint_on_other_side(self, q, obs):
        pts_list = self.kdl_arm.arm_config_to_points_list(q)
        if obs.shape[0] == 2:
            obs = np.row_stack((obs, np.matrix(pts_list[0][2])))
        pt_proj = hg.project_point_on_curve(obs, pts_list)

        pt_proj[2,0] = obs[2,0]
        v = obs-pt_proj
        v = v/np.linalg.norm(v)
        waypoint = obs + v * 0.04
        return waypoint

    def obstacles_within_distance(self, q, dist, obs_coord_list):
        obs_mat = np.matrix(obs_coord_list).T
        p_l = self.kdl_arm.arm_config_to_points_list(q)
        dist_arr = np.array([hg.distance_from_curve(np.matrix(obs).T, p_l) for obs in obs_coord_list])
        idxs = np.where(dist_arr<dist)[0]
        close_obs_mat = obs_mat[:,idxs]
        return close_obs_mat.T.tolist()

    def order_obstacles_along_arm(self, q, obs_coord_list):
        close_obs_l = self.obstacles_within_distance(q, 0.05, obs_coord_list)
        ee, _ = self.kdl_arm.FK(q)
        p_l = self.kdl_arm.arm_config_to_points_list(q)
        
        print close_obs_l
        for o in close_obs_l:
            proj = hg.project_point_on_curve(np.matrix(o).T, p_l)
            if np.allclose(proj[0:2], ee[0:2]):
                close_obs_l.remove(o)

        if close_obs_l == []:
            return []

        close_obs_mat = np.matrix(close_obs_l).T
        dist_along_arm_l = [hg.distance_along_curve(close_obs_mat[:,i], p_l)
                            for i in range(close_obs_mat.shape[1])]

        comb = zip(dist_along_arm_l, close_obs_l)
        comb.sort()
        comb.reverse()
        dist_along_arm_l, close_obs_l = zip(*comb)
        return close_obs_l

    def pick_obstacle_to_switch_sides(self, q, obs_coord_list):
        close_obs_l = self.order_obstacles_along_arm(q, obs_coord_list)
        side_list = self.assign_side_to_obstacles(q, close_obs_l)
        for i, o in enumerate(close_obs_l):
            o = np.matrix(o).T
            if not self.is_prev_attempted_switch(q, o, close_obs_l):
                return o
#            side_list[i] = not side_list[i]
#            if not self.is_prev_done_state(side_list, close_obs_l):
#                return o
            side_list[i] = not side_list[i]
        raise RuntimeError('Out of ideas')

    #-------- plotting and visualization -------
    def plot_polygon(self, po):
        pp.plot(*po.exterior.xy)

    # obs_coord_list - list of 2x1 or 3x1 np matrices.
    def plot_obstacles_with_side(self, obs_coord_list, obs_side_list):
        os_arr = np.array(obs_side_list)
        idxs_left = np.where(os_arr)[0]
        idxs_right = np.where(os_arr==False)[0]
        oc = np.array(obs_coord_list)[:,0:2]
        obs_left = oc[idxs_left]
        obs_right = oc[idxs_right]

        pp.plot(obs_left[:,0].flatten(), obs_left[:,1].flatten(),
                'go', ms=8, mew=0, label='left')
        pp.plot(obs_right[:,0].flatten(), obs_right[:,1].flatten(),
                'co', ms=8, mew=0, label='right')
        pp.legend()

    def check_obstacle_ordering(self, q, obs_coord_list):
        c_l = ['r', 'g', 'b', 'y', 'c', 'k', 'm', 'w']
        obs_l = self.order_obstacles_along_arm(q, obs_coord_list)
        for i,o in enumerate(obs_l):
            if i >= len(c_l):
                break
            pp.plot([o[0]], [o[1]], c_l[i]+'^', ms=8)



if __name__ =='__main__':
    import optparse
    import software_simulation.ode_sim_arms as osa
    import hrl_lib.util as ut

    p = optparse.OptionParser()
    p.add_option('--pkl', action='store', dest='pkl', type='string',
                 help='pkl with obstacle locations (for testing).')
    opt, args = p.parse_args()

    kdl_arm = osa.RobotSimulatorKDL()
    side_ana = ObstacleSideAnalyzer(kdl_arm)

    q = np.radians([-60, 120, 60])
    po = side_ana.polygon_from_config(q)
    #side_ana.plot_polygon(po)
    kdl_arm.plot_arm(q)

    if opt.pkl:
        d = ut.load_pickle(opt.pkl)
        n_fixed = d['num_fixed_used']
        fixed_pos = d['fixed_position'][0:n_fixed]
        obs_coord_list = fixed_pos
        side_list = side_ana.assign_side_to_obstacles(q, obs_coord_list)
        side_ana.plot_obstacles_with_side(obs_coord_list, side_list)
        side_ana.check_obstacle_ordering(q, obs_coord_list)
        

#    coords = [(0,0), (1,0), (1,0.5), (0, -0.5)]
#    po = sg.Polygon(coords)
#    plot_polygon(po)

    pp.axis('equal')
    pp.show()


