#!/usr/bin/python

import sys
import numpy as np, math

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.util as ut


def generate_far_locations(start_y, total_num):
    x = np.zeros(total_num)
    y = np.arange(start_y*1., start_y+total_num)
    return x, y

def in_collision(x, y, x_arr, y_arr, radius, goal):
    for i in range(x_arr.size):
        if np.linalg.norm([x_arr[i]-x, y_arr[i]-y]) < 2.*radius:
            return True
    if np.linalg.norm([x-goal[0,0], y-goal[1,0]]) < 2.*radius:
        return True
    return False

# random obstacles (fixed and moveable)
def generate_random_obstacles(x_lim, y_lim, num_move_used, num_compliant_used,
                              num_fixed_used, total_num, radius, goal):
    z = np.ones(total_num) * 0.0

    x_move, y_move = generate_far_locations(-100, total_num)
    for i in range(num_move_used):
        collision = True
        while collision:
            x = np.random.uniform(x_lim[0], x_lim[1], 1)
            y = np.random.uniform(y_lim[0], y_lim[1], 1)
            collision = in_collision(x, y, x_move, y_move, radius, goal)

        x_move[i] = x
        y_move[i] = y

    moveable_position = np.row_stack((x_move,y_move,z)).T.tolist()

    x_compliant, y_compliant = generate_far_locations(-200, total_num)
    for i in range(num_compliant_used):
        collision = True
        while collision:
            x = np.random.uniform(x_lim[0], x_lim[1], 1)
            y = np.random.uniform(y_lim[0], y_lim[1], 1)
            collision = in_collision(x, y, x_compliant, y_compliant, radius, goal) \
                        or in_collision(x, y, x_move, y_move, radius, goal)

        x_compliant[i] = x
        y_compliant[i] = y

    compliant_position = np.row_stack((x_compliant,y_compliant,z)).T.tolist()

    x_fix, y_fix = generate_far_locations(100, total_num)
    for i in xrange(num_fixed_used):
        collision = True
        while collision:
            x = np.random.uniform(x_lim[0], x_lim[1], 1)
            y = np.random.uniform(y_lim[0], y_lim[1], 1)
            collision = in_collision(x, y, x_fix, y_fix, radius, goal) \
                     or in_collision(x, y, x_move, y_move, radius, goal) \
                     or in_collision(x, y, x_compliant, y_compliant, radius, goal)

        x_fix[i] = x
        y_fix[i] = y

    fixed_position = np.row_stack((x_fix, y_fix, z)).T.tolist()
    return moveable_position, compliant_position, fixed_position

# biased random obstacles (fixed and moveable)
def generate_biased_random_obstacles(x_lim, y_lim, num_move_used, num_compliant_used,
                              num_fixed_used, total_num, radius, goal):
    z = np.ones(total_num) * 0.0

    x_move, y_move = generate_far_locations(-100, total_num)

    for i in range(num_move_used):
        collision = True
        while collision:
            x = np.random.uniform(x_lim[0], x_lim[1], 1)
            y = np.random.uniform(y_lim[0], y_lim[1], 1)
            collision = in_collision(x, y, x_move, y_move, radius, goal)

        x_move[i] = x
        y_move[i] = y

    moveable_position = np.row_stack((x_move,y_move,z)).T.tolist()

    x_compliant, y_compliant = generate_far_locations(-200, total_num)
    for i in range(num_compliant_used):
        collision = True
        while collision:
            x = np.random.uniform(x_lim[0], x_lim[1], 1)
            y = np.random.uniform(y_lim[0], y_lim[1], 1)
            collision = in_collision(x, y, x_compliant, y_compliant, radius, goal) \
                        or in_collision(x, y, x_move, y_move, radius, goal)

        x_compliant[i] = x
        y_compliant[i] = y

    compliant_position = np.row_stack((x_compliant,y_compliant,z)).T.tolist()


    fRndX = np.random.random(1)*0.4
    fRndY = np.random.random(1)*0.5+0.5
    #print fRndX,fRndY
    #fRnd = 0.3

    x_fix, y_fix = generate_far_locations(100, total_num)
    for i in xrange(num_fixed_used):
        collision = True
        while collision:
            #x = np.random.normal((x_lim[1]-x_lim[0])*fRndX+x_lim[0], 0.12 , 1)  
            #x = np.random.uniform(x_lim[0], x_lim[1], 1)

            fRndX = np.random.random(1)*0.2
            fRndY = np.random.random(1)*0.3+0.2
            print fRndX,fRndY

            x = (x_lim[1]-x_lim[0])*fRndX+x_lim[0]
            y = (y_lim[1]-y_lim[0])*fRndY+y_lim[0]

            #y = np.random.normal((y_lim[1]-y_lim[0])*fRndY+y_lim[0], 0.12 , 1)    
            print "map",x,y

            #y = np.random.uniform(y_lim[0], y_lim[1], 1)
            if y<y_lim[1] and y>y_lim[0] :
                #print x , y , (y_lim[1]-y_lim[0])*fRnd+y_lim[0] , y_lim[0] , y_lim[1]
                collision = in_collision(x, y, x_fix, y_fix, radius, goal) \
                    or in_collision(x, y, x_move, y_move, radius, goal) \
                    or in_collision(x, y, x_compliant, y_compliant, radius, goal)

        x_fix[i] = x
        y_fix[i] = y

    fixed_position = np.row_stack((x_fix, y_fix, z)).T.tolist()
    return moveable_position, compliant_position, fixed_position


def upload_to_param_server(d):
    rospy.set_param('m3/software_testbed/goal', d['goal'])
    rospy.set_param('m3/software_testbed/num_total', d['num_total'])

    rospy.set_param('m3/software_testbed/fixed_dimen', d['fixed_dimen'])
    rospy.set_param('m3/software_testbed/fixed_position', d['fixed_position'])
    rospy.set_param('m3/software_testbed/num_fixed', d['num_fixed_used'])

    rospy.set_param('m3/software_testbed/compliant_dimen', d.get('compliant_dimen', []))
    rospy.set_param('m3/software_testbed/compliant_position', d.get('compliant_position', []))
    rospy.set_param('m3/software_testbed/num_compliant', d.get('num_compliant_used', 0))
    rospy.set_param('m3/software_testbed/compliant_stiffness_value', d.get('stiffness_value',[]))

    rospy.set_param('/m3/software_testbed/movable_max_force', d.get('moveable_max_force', [2.0]*d['num_move_used']))
    rospy.set_param('m3/software_testbed/num_movable', d['num_move_used'])
    rospy.set_param('m3/software_testbed/movable_position', d['moveable_position'])
    # this needs to be the last param to be sent to the parameter server because of
    # stupid synchronization with demo_kinematic.cpp and draw_bodies.py
    rospy.set_param('m3/software_testbed/movable_dimen', d['moveable_dimen'])

def dict_from_param_server():
    d = {}
    d['fixed_dimen'] = rospy.get_param('m3/software_testbed/fixed_dimen')
    d['fixed_position'] = rospy.get_param('m3/software_testbed/fixed_position')
    d['num_fixed_used'] = rospy.get_param('m3/software_testbed/num_fixed')
    d['num_move_used'] = rospy.get_param('m3/software_testbed/num_movable')
    d['num_total'] = rospy.get_param('m3/software_testbed/num_total')
    d['goal'] = rospy.get_param('m3/software_testbed/goal')
    d['moveable_dimen'] = rospy.get_param('m3/software_testbed/movable_dimen')
    d['moveable_position'] = rospy.get_param('m3/software_testbed/movable_position')
    return d


if __name__ == '__main__':
    import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
    import rospy

    import optparse
    p = optparse.OptionParser()

    p.add_option('--fixed', action='store', dest='fixed',type='int',
                 default=0, help='number of fixed obstacles')
    p.add_option('--sliding', action='store', dest='sliding',type='int',
                 default=0, help='number of sliding obstacles')
    p.add_option('--compliant', action='store', dest='compliant',type='int',
                 default=0, help='number of compliant obstacles')

    p.add_option('--stiffness_value', action='store', dest='sv',
                 type='float', default=100, help='stiffness value for compliant obstacles')
    p.add_option('--sliding_max_force', action='store', dest='sliding_max_force',type='float',
                 default=2.0, help='max force for sliding obstacles')


    p.add_option('--xmin', action='store', dest='xmin',type='float',
                 default=0.2, help='min x coord for random obstacles')
    p.add_option('--xmax', action='store', dest='xmax',type='float',
                 default=0.6, help='max x coord for random obstacles')
    p.add_option('--ymin', action='store', dest='ymin',type='float',
                 default=-0.3, help='min y coord for random obstacles')
    p.add_option('--ymax', action='store', dest='ymax',type='float',
                 default=0.3, help='max y coord for random obstacles')

    p.add_option('--radius', action='store', dest='radius',type='float',
                 default=0.01, help='radius of the obstacles')

    p.add_option('--check_openrave', action='store_true', dest='co',
                 help='regenerate if openrave does not find a solution')

    p.add_option('--save_pkl', action='store_true', dest='s_pkl',
                 help='save config as pkl, instead of on the param server')
    p.add_option('--pkl', action='store', dest='pkl', default=None,
                 help='pkl to read and load to the param server')
    p.add_option('--get_param_server', action='store_true',
                 dest='gps',
                 help='get params from param server and save as pkl')

    p.add_option('--add_stuff', action='store', dest='add_stuff', default=None,
                 help='add X number of obstacles to pickle, needs manual tweaking in file as well')

    p.add_option('--random', action='store', dest='rand_option', type='int', default=0,
                 help='Input random option: 0=Uniform, 1=Biased')

    opt, args = p.parse_args()

    total_num = 1000
    radius = opt.radius

    moveable_dimen = [[radius, radius, 0.2] for i in range(total_num)]
    fixed_dimen = [[radius, radius, 0.2] for i in range(total_num)]
    compliant_dimen = [[radius, radius, 0.2] for i in range(total_num)]

    x_goal = float(np.random.uniform(opt.xmin, opt.xmax, 1)[0])
    y_goal = float(np.random.uniform(opt.ymin, opt.ymax, 1)[0]) 
    goal = np.matrix([x_goal, y_goal, 0]).T
    
    x_lim = [opt.xmin, opt.xmax]
    y_lim = [opt.ymin, opt.ymax]

    if opt.rand_option == 1 :
        sliding_pos, compliant_pos, fixed_pos = generate_biased_random_obstacles(x_lim,
                                                                          y_lim, opt.sliding, opt.compliant, opt.fixed,
                                                                          total_num, radius, goal)
    else:
        sliding_pos, compliant_pos, fixed_pos = generate_random_obstacles(x_lim,
                                                                          y_lim, opt.sliding, opt.compliant, opt.fixed,
                                                                          total_num, radius, goal)
        
    stiff_ls = [opt.sv] * opt.compliant

    if opt.pkl != None and opt.add_stuff != None:

        x_new = []
        y_new = []
        move_pos = np.array(d['moveable_position'])
        fixed_pos = np.array(d['fixed_position'])
        print "size of move_pos is: ", move_pos.shape
        print move_pos[:,0]
        z_new = np.ones(opt.add_stuff) * 0.0
        for i in xrange(opt.add_stuff):
            collision = True
            while collision:
                if xmin == None or xmax == None or ymin == None or ymax == None:
                    print "need to specify x, y max and min for this argument"
                x = np.random.uniform(xmin, xmax, 1)
                y = np.random.uniform(ymin, ymax, 1)
                
                collision = in_collision(x, y, move_pos[:,0], move_pos[:,1], radius, goal) \
                    or in_collision(x, y, fixed_pos[:,0], fixed_pos[:,1], radius, goal)

            x_new.append(x[0])
            y_new.append(y[0])

        movable_position = np.row_stack((np.array(x_new), np.array(y_new), z_new)).T.tolist()

        #this part needs to be cleaned up and made more general!
        d['moveable_position'][11] = movable_position[0]
        d['moveable_position'][12] = movable_position[1]
        d['moveable_position'][13] = movable_position[2]
        d['moveable_position'][14] = movable_position[3]
        d['moveable_position'][15] = movable_position[4]
        d['goal'] =  goal.A1.tolist()
        upload_to_param_server(d)
    elif opt.pkl != None:
        d = ut.load_pickle(opt.pkl)        
        upload_to_param_server(d)
    else:

        if opt.gps:
            d = dict_from_param_server()
            ut.save_pickle(d, 'reach_problem_dict.pkl')
        else:
            d = {}
            d['fixed_dimen'] = fixed_dimen
            d['fixed_position'] = fixed_pos
            d['num_fixed_used'] = opt.fixed

            d['num_compliant_used'] = opt.compliant
            d['compliant_dimen'] = compliant_dimen
            d['compliant_position'] = compliant_pos
            d['stiffness_value'] = stiff_ls

            d['num_move_used'] = opt.sliding
            d['moveable_dimen'] = moveable_dimen
            d['moveable_position'] = sliding_pos
            d['moveable_max_force'] = [opt.sliding_max_force]*opt.sliding

            d['num_total'] = total_num
            d['goal'] = goal.A1.tolist()

            if opt.co:
                # check openrave.
                print 'Calling OpenRAVE to ceck if path exists'
                import geometric_search.planar_openrave as gspo
                res = gspo.setup_openrave_and_plan(d, True, True,
                        'openrave_result_ignore_moveable.pkl')
                print 'OpenRAVE result:', res
                if not res:
                    sys.exit(1)

            if opt.s_pkl:
                ut.save_pickle(d, 'reach_problem_dict.pkl')
            else:
                upload_to_param_server(d)



