#!/usr/bin/python

import sys
import numpy as np, math
import random

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.util as ut


def generate_far_locations(start_y, total_num):
    x = np.zeros(total_num)
    y = np.arange(start_y*1., start_y+total_num)
    z = np.zeros(total_num)
    s = np.zeros(total_num)
    return x, y, z, s

# Maybe useless - dpark 20130715
def in_collision(x, y, s, x_arr, y_arr, s_arr, ctype, dim, goal):

    if ctype == 'wall':
        length = dim[0]        
        width  = dim[1]
        slope  = dim[3]

        # Not completed - dpark 20130404
        for i in range(x_arr.size):
            if (np.linalg.norm([x_arr[i]-x]) < width and
                np.linalg.norm([y_arr[i]-y]) < length):
                return True
        if (np.linalg.norm([x-goal[0,0]]) < width/2.0 and
            np.linalg.norm([y-goal[1,0]]) < length/2.0):
            return True        
    else:
        radius = dim[0]
    
        for i in range(x_arr.size):
            if np.linalg.norm([x_arr[i]-x, y_arr[i]-y]) < 2.*radius:
                return True
        if np.linalg.norm([x-goal[0,0], y-goal[1,0]]) < 2.*radius:
            return True
        
    return False

def in_collision_set(dim, pos, ctype, dim_arr, pos_arr, ctype_list):

    if len(ctype_list) == 0: return False

    for i in range(len(ctype_list)):

        if ctype == 'cylinder':

            if ctype_list[i] == 'cylinder':
                if np.linalg.norm([pos_arr[i][0]-pos[0], pos_arr[i][1]-pos[1]]) < dim[0] + dim_arr[i][0]:
                    return True

            elif ctype_list[i] == 'wall':

                slope  = pos_arr[i][3]
                mRot   = np.matrix([[np.cos(slope), -np.sin(slope)],
                                  [np.sin(slope), np.cos(slope)]])

                length = dim_arr[i][0]
                width  = dim_arr[i][1]            
                mEdge  = np.matrix([[-length/2.0, length/2.0],
                                    [0.0, 0.0]])
                mNewEdge = mRot*mEdge
                mNewEdge += np.matrix([[pos_arr[i][0], pos_arr[i][0]],
                                       [pos_arr[i][1], pos_arr[i][1]]])

                if dist_segment_point(mNewEdge[0,0],mNewEdge[1,0], mNewEdge[0,1],mNewEdge[1,1], pos[0],pos[1]) < dim[0] + width:
                    return True

        elif ctype == 'wall':
            
            if ctype_list[i] == 'cylinder':

                slope  = pos[3]
                mRot   = np.matrix([[np.cos(slope), -np.sin(slope)],
                                  [np.sin(slope), np.cos(slope)]])

                length = dim[0]
                width  = dim[1]
                mEdge  = np.matrix([[-length/2.0, length/2.0],
                                    [0.0, 0.0]])
                mNewEdge = mRot*mEdge
                mNewEdge += np.matrix([[pos[0], pos[0]],
                                       [pos[1], pos[1]]])

                if dist_segment_point(mNewEdge[0,0],mNewEdge[1,0], mNewEdge[0,1],mNewEdge[1,1], pos_arr[i][0],pos_arr[i][1]) < width + dim_arr[i][1]:
                    return True
            
            elif ctype_list[i] == 'wall':

                # First segment
                slope  = pos[3]
                mRot   = np.matrix([[np.cos(slope), -np.sin(slope)],
                                  [np.sin(slope), np.cos(slope)]])

                length = dim[0]
                width  = dim[1]
                wdist  = dim[4]
                mEdge  = np.matrix([[-length/2.0, length/2.0],
                                    [0.0, 0.0]])
                mNewEdge_1 = mRot*mEdge
                mNewEdge_1 += np.matrix([[pos[0], pos[0]],
                                         [pos[1], pos[1]]])

                # Second segment
                slope  = pos_arr[i][3]
                mRot   = np.matrix([[np.cos(slope), -np.sin(slope)],
                                  [np.sin(slope), np.cos(slope)]])

                length = dim_arr[i][0]
                width  = dim_arr[i][1]            
                mEdge  = np.matrix([[-length/2.0, length/2.0],
                                    [0.0, 0.0]])
                mNewEdge_2 = mRot*mEdge
                mNewEdge_2 += np.matrix([[pos_arr[i][0], pos_arr[i][0]],
                                         [pos_arr[i][1], pos_arr[i][1]]])

                if distance_segments(mNewEdge_1[0,0],mNewEdge_1[1,0], mNewEdge_1[0,1],mNewEdge_1[1,1],
                                     mNewEdge_2[0,0],mNewEdge_2[1,0], mNewEdge_2[0,1],mNewEdge_2[1,1]) < wdist:
                    return True
                
        elif ctype == 'sphere':
            
            if ctype_list[i] == 'sphere':

                if np.linalg.norm([pos_arr[i][0]-pos[0], pos_arr[i][1]-pos[1], pos_arr[i][2]-pos[2]]) < dim[0] + dim_arr[i][0]:
                    return True
                
            
            
    return False

# x1,y1 is start of segment
# x2,y2 is end of segment
# x3,y3 is a point    
def dist_segment_point(x1,y1, x2,y2, x3,y3): 
    px = x2-x1
    py = y2-y1

    length = px*px + py*py

    u =  ((x3 - x1) * px + (y3 - y1) * py) / float(length)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    x = x1 + u * px
    y = y1 + u * py

    dx = x - x3
    dy = y - y3

    dist = np.sqrt(dx*dx + dy*dy)

    return dist    

# Decide two segments intersect or not
# First segment is (x11, y11) - (x12, y12)
# Second segment is (x21, y21) - (x22, y22)
def intersect_segments(x11, y11, x12, y12, x21, y21, x22, y22):
    dx1 = x12 - x11
    dy1 = y12 - y11
    dx2 = x22 - x21
    dy2 = y22 - y21
    delta = dx2 * dy1 - dy2 * dx1
    if delta == 0: return False  # parallel segments
    s = (dx1 * (y21 - y11) + dy1 * (x11 - x21)) / delta
    t = (dx2 * (y11 - y21) + dy2 * (x21 - x11)) / (-delta)
    return (0 <= s <= 1) and (0 <= t <= 1)
    
# Calculate distance between two segments
# First segment is (x11, y11) - (x12, y12)
# Second segment is (x21, y21) - (x22, y22)    
# Ref: http://stackoverflow.com/questions/2824478/shortest-distance-between-two-line-segments
def distance_segments(x11, y11, x12, y12, x21, y21, x22, y22):
    if intersect_segments(x11, y11, x12, y12, x21, y21, x22, y22): return 0
    # try each of the 4 vertices w/the other segment
    distances = []
    distances.append(dist_segment_point(x21, y21, x22, y22, x11, y11))
    distances.append(dist_segment_point(x21, y21, x22, y22, x12, y12))
    distances.append(dist_segment_point(x11, y11, x12, y12, x21, y21))
    distances.append(dist_segment_point(x11, y11, x12, y12, x22, y22))
    return min(distances)

    
    
# random obstacles (fixed and moveable)
def generate_random_obstacles(x_lim, y_lim, z_lim, s_lim, num_move_used, num_compliant_used,
                              num_fixed_used, total_num, ctype, dim, multi_obstacle=False):

    dim_arr = []
    pos_arr = []
    ctype_list = []
    
    if multi_obstacle:
        d = ut.load_pickle('reach_problem_dict.pkl')

        if d['num_fixed_used'] > 0:                
            dim_arr += d['fixed_dimen'] 
            pos_arr += d['fixed_position'] 
            ctype_list += d.get('fixed_ctype', ['cylinder']*d['num_fixed_used'])

        if d['num_compliant_used'] > 0:                
            dim_arr += d['compliant_dimen'] 
            pos_arr += d['compliant_position'] 
            ctype_list += d.get('compliant_ctype', ['cylinder']*d['num_compliant_used'])

        if d['num_move_used'] > 0:                
            dim_arr += d['moveable_dimen'] 
            pos_arr += d['moveable_position'] 
            ctype_list += d.get('moveable_ctype', ['cylinder']*d['num_move_used'])
            
    #z = np.ones(num_move_used) * 0.0
    x_move, y_move, z_move, s_move = generate_far_locations(-100, num_move_used)
    
    for i in range(num_move_used):
        collision = True
        while collision:
                
            x = np.random.uniform(x_lim[0], x_lim[1], 1)[0]
            y = np.random.uniform(y_lim[0], y_lim[1], 1)[0]
            z = np.random.uniform(z_lim[0], z_lim[1], 1)[0]            
            s = np.random.uniform(s_lim[0], s_lim[1], 1)[0] # slope                
                
            pos = [x,y,z,s]
            collision = in_collision_set(dim, pos, ctype, dim_arr, pos_arr, ctype_list)
            #collision = in_collision(x, y, s, x_move, y_move, s_move, ctype, dim, goal)

        dim_arr.append(dim)
        pos_arr.append(pos)
        ctype_list.append(ctype)

        x_move[i] = x
        y_move[i] = y
        z_move[i] = z
        s_move[i] = s
        
    moveable_position = np.row_stack((x_move,y_move,z_move,s_move)).T.tolist()

    #z = np.ones(num_compliant_used) * 0.0    
    x_compliant, y_compliant, z_compliant, s_compliant = generate_far_locations(-200, num_compliant_used)
    for i in range(num_compliant_used):
        collision = True
        while collision:
            
            x = np.random.uniform(x_lim[0], x_lim[1], 1)[0]
            y = np.random.uniform(y_lim[0], y_lim[1], 1)[0]
            z = np.random.uniform(z_lim[0], z_lim[1], 1)[0]            
            s = np.random.uniform(s_lim[0], s_lim[1], 1)[0] # slope                
                
            pos = [x,y,z,s]
            collision = in_collision_set(dim, pos, ctype, dim_arr, pos_arr, ctype_list)
                
            ## collision = in_collision(x, y, s, x_compliant, y_compliant, s_compliant, radius, goal) \
            ##             or in_collision(x, y, s, x_move, y_move, s_move, ctype, dim, goal)
        dim_arr.append(dim)
        pos_arr.append(pos)
        ctype_list.append(ctype)
            
        x_compliant[i] = x
        y_compliant[i] = y
        z_compliant[i] = z
        s_compliant[i] = s

    compliant_position = np.row_stack((x_compliant,y_compliant,z_compliant,s_compliant)).T.tolist()

    #z = np.ones(num_fixed_used) * 0.0        
    x_fix, y_fix, z_fix, s_fix = generate_far_locations(100, num_fixed_used)
    for i in xrange(num_fixed_used):
        collision = True
        while collision:
            
            x = np.random.uniform(x_lim[0], x_lim[1], 1)[0]
            y = np.random.uniform(y_lim[0], y_lim[1], 1)[0]
            z = np.random.uniform(z_lim[0], z_lim[1], 1)[0]            
            s = np.random.uniform(s_lim[0], s_lim[1], 1)[0] # slope                
                
            pos = [x,y,z,s]
            collision = in_collision_set(dim, pos, ctype, dim_arr, pos_arr, ctype_list)
            ## collision = in_collision(x, y, s, x_fix, y_fix, s_fix, ctype, dim, goal) \
            ##          or in_collision(x, y, s, x_move, y_move, s_move, ctype, dim, goal) \
            ##          or in_collision(x, y, s, x_compliant, y_compliant, s_compliant, ctype, dim, goal)
        dim_arr.append(dim)
        pos_arr.append(pos)
        ctype_list.append(ctype)

        x_fix[i] = x
        y_fix[i] = y
        z_fix[i] = z        
        s_fix[i] = s

    fixed_position = np.row_stack((x_fix, y_fix, z_fix, s_fix)).T.tolist()
    
    return moveable_position, compliant_position, fixed_position

# random obstacle unit (fixed and moveable)
def generate_random_obstacle_unit(x_lim, y_lim, z_lim, s_lim, num_move_used, num_compliant_used,
                                  num_fixed_used, total_num, unit_type, dim, multi_obstacle=False):

    dim_arr = []
    pos_arr = []
    ctype_list = []

    if unit_type == 'passage' or unit_type == 'window': ctype = 'wall'
    
    if multi_obstacle:
        d = ut.load_pickle('reach_problem_dict.pkl')

        if d['num_fixed_used'] > 0:                
            dim_arr += d['fixed_dimen'] 
            pos_arr += d['fixed_position'] 
            ctype_list += d.get('fixed_ctype', ['cylinder']*d['num_fixed_used'])

        if d['num_compliant_used'] > 0:                
            dim_arr += d['compliant_dimen'] 
            pos_arr += d['compliant_position'] 
            ctype_list += d.get('compliant_ctype', ['cylinder']*d['num_compliant_used'])

        if d['num_move_used'] > 0:                
            dim_arr += d['moveable_dimen'] 
            pos_arr += d['moveable_position'] 
            ctype_list += d.get('moveable_ctype', ['cylinder']*d['num_move_used'])
            
    z = np.ones(num_move_used) * 0.0
    x_move, y_move, z_move, s_move = generate_far_locations(-100, num_move_used)
    
    for i in range(int(num_move_used/2.0)):
        collision = True
        while collision:
            if unit_type == 'passage':               
                x = np.random.uniform(x_lim[0], x_lim[1], 1)[0]
                y = np.random.uniform(y_lim[0], y_lim[1], 1)[0]
                z = np.random.uniform(z_lim[0], z_lim[1], 1)[0]                
                s = np.random.uniform(s_lim[0], s_lim[1], 1)[0] # slope
            else:
                x = np.random.uniform(x_lim[0], x_lim[1], 1)
                y = np.random.uniform(y_lim[0], y_lim[1], 1)                
                s = 0.0

            pos = [x,y,z,s]
            collision = in_collision_set(dim, pos, ctype, dim_arr, pos_arr, ctype_list)
            #collision = in_collision(x, y, s, x_move, y_move, s_move, ctype, dim, goal)

        dim_arr.append(dim)
        pos_arr.append(pos)
        ctype_list.append(ctype)

        x_move[i] = x
        y_move[i] = y
        z_move[i] = z        
        s_move[i] = s
        
    moveable_position = np.row_stack((x_move,y_move,z_move,s_move)).T.tolist()

    ## z = np.ones(num_compliant_used) * 0.0    
    x_compliant, y_compliant, z_compliant, s_compliant = generate_far_locations(-200, num_compliant_used)
    for i in range(int(num_compliant_used/2.0)):
        collision = True
        while collision:
            if unit_type == 'passage':                          
                x = np.random.uniform(x_lim[0], x_lim[1], 1)[0]
                y = np.random.uniform(y_lim[0], y_lim[1], 1)[0]
                z = np.random.uniform(z_lim[0], z_lim[1], 1)[0]                                
                s = np.random.uniform(s_lim[0], s_lim[1], 1)[0] # slope                
            else:
                x = np.random.uniform(x_lim[0], x_lim[1], 1)
                y = np.random.uniform(y_lim[0], y_lim[1], 1)                
                s = 0.0

            pos = [x,y,z,s]
            collision = in_collision_set(dim, pos, ctype, dim_arr, pos_arr, ctype_list)
                
            ## collision = in_collision(x, y, s, x_compliant, y_compliant, s_compliant, radius, goal) \
            ##             or in_collision(x, y, s, x_move, y_move, s_move, ctype, dim, goal)
        dim_arr.append(dim)
        pos_arr.append(pos)
        ctype_list.append(ctype)
            
        x_compliant[i] = x
        y_compliant[i] = y
        z_compliant[i] = z        
        s_compliant[i] = s

    compliant_position = np.row_stack((x_compliant,y_compliant,z_compliant,s_compliant)).T.tolist()

    ## z = np.ones(num_fixed_used) * 0.0        
    x_fix, y_fix, z_fix, s_fix = generate_far_locations(100, num_fixed_used)

    for i in xrange(int(num_fixed_used/2.0)):
        collision = True
        count = 0
        while collision:
            count += 1

            if count > 50:
                return None, None, None, False
                
            if unit_type == 'passage':                          

                x = np.random.uniform(x_lim[0], x_lim[1], 1)[0] 
                y = np.random.uniform(y_lim[0], y_lim[1], 1)[0]
                z = np.random.uniform(z_lim[0], z_lim[1], 1)[0]                
                s = np.random.uniform(s_lim[0], s_lim[1], 1)[0] # slope                

                width = 0.12

                # Positive normal
                nrl = np.matrix([[np.cos(s+np.pi/2.0), -np.sin(s+np.pi/2.0)],
                                 [np.sin(s+np.pi/2.0), np.cos(s+np.pi/2.0)]])
                x1 = x + (nrl*np.matrix([[width/2.0],[0.]]))[0,0]
                y1 = y + (nrl*np.matrix([[width/2.0],[0.]]))[1,0]

                nrl = np.matrix([[np.cos(s-np.pi/2.0), -np.sin(s-np.pi/2.0)],
                                 [np.sin(s-np.pi/2.0), np.cos(s-np.pi/2.0)]])

                x2 = x + (nrl*np.matrix([[width/2.0],[0.]]))[0,0]
                y2 = y + (nrl*np.matrix([[width/2.0],[0.]]))[1,0]                

                pos1 = [x1,y1,z,s]
                collision1 = in_collision_set(dim, pos1, ctype, dim_arr, pos_arr, ctype_list)

                pos2 = [x2,y2,z,s]
                collision2 = in_collision_set(dim, pos2, ctype, dim_arr, pos_arr, ctype_list)

                if collision1 == False and collision2 == False: 
                    collision = False
                    print "Pass!!"
            elif unit_type == 'window':

                x = np.random.uniform(x_lim[0], x_lim[1], 1)[0] 
                y = np.random.uniform(y_lim[0], y_lim[1], 1)[0]
                z = np.random.uniform(z_lim[0], z_lim[1], 1)[0]                
                s = np.random.uniform(s_lim[0], s_lim[1], 1)[0] # slope                

                width = 0.12

                # Positive normal
                nrl = np.matrix([[np.cos(s+np.pi/2.0), -np.sin(s+np.pi/2.0)],
                                 [np.sin(s+np.pi/2.0), np.cos(s+np.pi/2.0)]])
                x1 = x + (nrl*np.matrix([[width/2.0],[0.]]))[0,0]
                y1 = y + (nrl*np.matrix([[width/2.0],[0.]]))[1,0]

                nrl = np.matrix([[np.cos(s-np.pi/2.0), -np.sin(s-np.pi/2.0)],
                                 [np.sin(s-np.pi/2.0), np.cos(s-np.pi/2.0)]])

                x2 = x + (nrl*np.matrix([[width/2.0],[0.]]))[0,0]
                y2 = y + (nrl*np.matrix([[width/2.0],[0.]]))[1,0]                

                pos1 = [x1,y1,z,s]
                collision1 = in_collision_set(dim, pos1, ctype, dim_arr, pos_arr, ctype_list)

                pos2 = [x2,y2,z,s]
                collision2 = in_collision_set(dim, pos2, ctype, dim_arr, pos_arr, ctype_list)

                if collision1 == False and collision2 == False: 
                    collision = False
                    print "Pass!!"


        dim_arr.append(dim)
        dim_arr.append(dim)
        
        pos_arr.append(pos1)
        pos_arr.append(pos2)
                
        ctype_list.append(ctype)
        ctype_list.append(ctype)        
        
        x_fix[i*2] = x1
        y_fix[i*2] = y1
        z_fix[i*2] = z        
        s_fix[i*2] = s

        x_fix[i*2+1] = x2
        y_fix[i*2+1] = y2
        z_fix[i*2+1] = z        
        s_fix[i*2+1] = s

    fixed_position = np.row_stack((x_fix, y_fix, z_fix, s_fix)).T.tolist()
    
    return moveable_position, compliant_position, fixed_position, True

# gate (fixed)    
def generate_gate(fixed_pos, fixed_dimen, fixed_ctype, gate_x, gate_ymin, gate_ymax, gate_zmin, gate_zmax, gate_offset, gate_blocks):

    fixed_num = 0
    
    # Left & right sides
    side_wall_length = 1.0
    gate_width       = 0.02
    fixed_dimen.append([side_wall_length, gate_width, gate_zmax, 0.0, 0.0])
    fixed_dimen.append([side_wall_length, gate_width, gate_zmax, 0.0, 0.0])
    fixed_pos.append([float(gate_x), float(gate_ymin-side_wall_length/2.0), float((gate_zmin+gate_zmax)/2.0), float(np.pi/2.0)])
    fixed_pos.append([float(gate_x), float(gate_ymax+side_wall_length/2.0), float((gate_zmin+gate_zmax)/2.0), float(np.pi/2.0)])
    fixed_ctype.append('wall')
    fixed_ctype.append('wall')        
    fixed_num += 2

    # Middle
    pole_list = np.arange(gate_ymin+gate_offset,gate_ymax,gate_offset)
    for i in xrange(len(pole_list)):
        fixed_dimen.append([gate_width/2.0,gate_width/2.0,gate_zmax,0.0,0.0])
        fixed_pos.append([float(gate_x), float(pole_list[i]), float((gate_zmin+gate_zmax)/2.0), float(np.pi/4.0)])
        fixed_ctype.append('cylinder')
        fixed_num += 1

    # Blocks
    TOL=0.00000001
    gate_pos_x = [gate_x]
    gate_pos_y = np.arange(gate_ymin+gate_offset/2.0,gate_ymax-gate_offset/2.0+TOL,gate_offset).tolist()
    gate_pos_z = [(gate_zmin+gate_zmax)/2.0]

    gate_pos = []
    for x in gate_pos_x:
        for y in gate_pos_y:
            for z in gate_pos_z:
                gate_pos.append([x,y,z])

    if gate_blocks > 0:
        # Randomly selects the index of gate_pos array
        block_pos = random.sample(gate_pos,gate_blocks)
        
        for pos in block_pos:
            fixed_dimen.append([gate_offset-gate_width,gate_width,gate_zmax,0.0,0.0])
            fixed_pos.append([float(gate_x), float(pos[1]), float(pos[2]), float(np.pi/2.0)])
            fixed_ctype.append('wall')
            fixed_num += 1

            gate_pos.remove(pos)
            
    return fixed_pos, fixed_dimen, fixed_ctype, fixed_num, gate_pos
    
    
def upload_to_param_server(d):
    
    rospy.set_param('m3/software_testbed/goal', d.get('goal', [[0.,0.,0.]]))
    rospy.set_param('m3/software_testbed/num_total', d['num_total'])

    rospy.set_param('m3/software_testbed/num_fixed', d['num_fixed_used'])    
    rospy.set_param('m3/software_testbed/fixed_dimen', d['fixed_dimen'])
    rospy.set_param('m3/software_testbed/fixed_position', d['fixed_position'])
    rospy.set_param('m3/software_testbed/fixed_ctype', d.get('fixed_ctype', ['cylinder']*d['num_fixed_used']))

    rospy.set_param('m3/software_testbed/num_compliant', d.get('num_compliant_used', 0))    
    rospy.set_param('m3/software_testbed/compliant_dimen', d.get('compliant_dimen', []))
    rospy.set_param('m3/software_testbed/compliant_position', d.get('compliant_position', []))
    rospy.set_param('m3/software_testbed/compliant_stiffness_value', d.get('stiffness_value',[]))

    rospy.set_param('m3/software_testbed/num_movable', d['num_move_used'])    
    rospy.set_param('m3/software_testbed/movable_max_force', d.get('moveable_max_force', [2.0]*d['num_move_used']))
    rospy.set_param('m3/software_testbed/movable_position', d['moveable_position'])
    # this needs to be the last param to be sent to the parameter server because of
    # stupid synchronization with demo_kinematic.cpp and draw_bodies.py
    rospy.set_param('m3/software_testbed/movable_dimen', d['moveable_dimen'])
    rospy.set_param('m3/software_testbed/movable_ctype', d.get('moveable_ctype', ['cylinder']*d['num_move_used']))
    
    rospy.set_param('m3/software_testbed/gate_position', d.get('gate_position', [[0.,0.,0.]]))

    
def dict_from_param_server():
    d = {}
    d['num_fixed_used'] = rospy.get_param('m3/software_testbed/num_fixed')        
    d['fixed_dimen']    = rospy.get_param('m3/software_testbed/fixed_dimen')
    d['fixed_position'] = rospy.get_param('m3/software_testbed/fixed_position')
    d['fixed_ctype']    = rospy.get_param('m3/software_testbed/fixed_ctype')
        
    d['num_move_used']     = rospy.get_param('m3/software_testbed/num_movable')
    d['moveable_dimen']    = rospy.get_param('m3/software_testbed/movable_dimen')
    d['moveable_position'] = rospy.get_param('m3/software_testbed/movable_position')
    d['moveable_ctype']    = rospy.get_param('m3/software_testbed/movable_ctype')
    
    d['num_total']      = rospy.get_param('m3/software_testbed/num_total')
    d['goal']           = rospy.get_param('m3/software_testbed/goal')
    d['gate_position'] = rospy.get_param('m3/software_testbed/gate_position')
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
    p.add_option('--zmin', action='store', dest='zmin',type='float',
                 default=0.0, help='min z coord for random obstacles')
    p.add_option('--zmax', action='store', dest='zmax',type='float',
                 default=0.0, help='max z coord for random obstacles')
    p.add_option('--smin', action='store', dest='smin',type='float',
                 default=-np.pi, help='min slope for random obstacles')
    p.add_option('--smax', action='store', dest='smax',type='float',
                 default=np.pi, help='max slope for random obstacles')

    
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

    p.add_option('--c', '--class_type', action='store', dest='ctype', type='string',
                 default='cylinder')
    p.add_option('--width', action='store', dest='width', type='float',
                 default=0.02, help='width of the obstacles')
    p.add_option('--length', action='store', dest='length', type='float',
                 default=0.6, help='length of the obstacles')
    p.add_option('--wdist', action='store', dest='wdist', type='float',
                 default=0.0, help='Minimum distance between wall obstacles. Default is 0.0')

    p.add_option('--gate', action='store_true', dest='gate', default=False,
                 help='Create gates around initial configarions')
    p.add_option('--gate_blocks', action='store', dest='gate_blocks',type='int',
                 default=0, help='number of blocked gates')    
    p.add_option('--gate_x', action='store', dest='gate_x',type='float',
                 default=0.2, help='x coord for gates')
    p.add_option('--gate_ymin', action='store', dest='gate_ymin',type='float',
                 default=-0.6, help='min y coord for gates')
    p.add_option('--gate_ymax', action='store', dest='gate_ymax',type='float',
                 default=0.6, help='max y coord for gates')
    p.add_option('--gate_zmin', action='store', dest='gate_zmin',type='float',
                 default=-0.2, help='min z coord for gates')
    p.add_option('--gate_zmax', action='store', dest='gate_zmax',type='float',
                 default=0.2, help='max z coord for gates')
    p.add_option('--gate_offset', action='store', dest='gate_offset',type='float',
                 default=0.1, help='offset between gates')
    
    
    

    opt, args = p.parse_args()

    #total_num = 1000
    total_num = opt.fixed + opt.compliant + opt.sliding
    
    # Obstacle type
    ctype  = opt.ctype    
    if ctype == 'wall' or ctype == 'passage' or ctype == 'window':
        length = opt.length
        width  = opt.width
        slope  = (opt.smin+opt.smax)/2.0   
        wdist  = opt.wdist 
        dim    = [length, width, 0.2, slope, wdist]
    else:
        radius = opt.radius
        dim    = [radius, radius, 0.8]

    moveable_dimen  = [dim for i in range(opt.sliding)]
    fixed_dimen     = [dim for i in range(opt.fixed)]
    compliant_dimen = [dim for i in range(opt.compliant)]

    moveable_ctype  = [ctype for i in range(opt.sliding)]
    fixed_ctype     = [ctype for i in range(opt.fixed)]
    compliant_ctype = [ctype for i in range(opt.compliant)]        

    # Goal (Maybe useless - dpark 20130327)
    ## x_goal = float(np.random.uniform(opt.xmin, opt.xmax, 1)[0])
    ## y_goal = float(np.random.uniform(opt.ymin, opt.ymax, 1)[0]) 
    ## goal = np.matrix([100000, 100000, 0]).T

    x_lim = [opt.xmin, opt.xmax]
    y_lim = [opt.ymin, opt.ymax]
    z_lim = [opt.zmin, opt.zmax]  
    s_lim = [opt.smin, opt.smax]
        
    # Use multi obstacles such as walls and cylinders
    if opt.s_pkl != None and opt.add_stuff != None:
        multi_obstacle = True
    else:
        multi_obstacle = False

    if ctype == 'passage' or ctype == 'window':
        while True:
            sliding_pos, compliant_pos, fixed_pos, res = generate_random_obstacle_unit(x_lim,
                                                                                      y_lim, z_lim, s_lim, opt.sliding, opt.compliant, opt.fixed,
                                                                                      total_num, ctype, dim, multi_obstacle)
            if res == True: break
        ctype = 'wall'
        fixed_ctype     = [ctype for i in range(opt.fixed)]

    else:
        sliding_pos, compliant_pos, fixed_pos = generate_random_obstacles(x_lim,
                                                                          y_lim, z_lim, s_lim, opt.sliding, opt.compliant, opt.fixed,
                                                                          total_num, ctype, dim, multi_obstacle)


    if opt.gate:
        fixed_pos, fixed_dimen, fixed_ctype, fixed_num, gate_pos = generate_gate(fixed_pos, fixed_dimen, fixed_ctype, opt.gate_x, opt.gate_ymin, opt.gate_ymax, opt.gate_zmin, opt.gate_zmax, opt.gate_offset, opt.gate_blocks)
        opt.fixed += fixed_num
        total_num += fixed_num
    else:
        gate_pos = [[0.,0.,0.]]
        
    stiff_ls = [opt.sv] * opt.compliant

    # Not support in this moment: 03/27/2013
    if opt.pkl != None and opt.add_stuff != None and opt.s_pkl == None:
        print "Not support command!"
        ## x_new = []
        ## y_new = []
        ## move_pos = np.array(d['moveable_position'])
        ## fixed_pos = np.array(d['fixed_position'])
        ## print "size of move_pos is: ", move_pos.shape
        ## z_new = np.ones(opt.add_stuff) * 0.0
        ## for i in xrange(opt.add_stuff):
        ##     collision = True
        ##     while collision:
        ##         if xmin == None or xmax == None or ymin == None or ymax == None:
        ##             print "need to specify x, y max and min for this argument"
        ##         x = np.random.uniform(xmin, xmax, 1)
        ##         y = np.random.uniform(ymin, ymax, 1)
                
        ##         collision = in_collision(x, y, move_pos[:,0], move_pos[:,1], ctype, dim, goal) \
        ##             or in_collision(x, y, fixed_pos[:,0], fixed_pos[:,1], ctype, dim, goal)

        ##     x_new.append(x[0])
        ##     y_new.append(y[0])

        ## movable_position = np.row_stack((np.array(x_new), np.array(y_new), z_new)).T.tolist()

        ## #this part needs to be cleaned up and made more general!
        ## d['moveable_position'][11] = movable_position[0]
        ## d['moveable_position'][12] = movable_position[1]
        ## d['moveable_position'][13] = movable_position[2]
        ## d['moveable_position'][14] = movable_position[3]
        ## d['moveable_position'][15] = movable_position[4]
        ## d['goal'] =  goal.A1.tolist()
        ## upload_to_param_server(d)
    elif opt.pkl != None:
        d = ut.load_pickle(opt.pkl)
        upload_to_param_server(d)
    elif opt.s_pkl != None and opt.add_stuff != None:
        d = ut.load_pickle('reach_problem_dict.pkl')

        ## pre_fixed_dimen = d['fixed_dimen']  
        ## pre_fixed_pos = d['fixed_position']  
        ## pre_fixed = d['num_fixed_used'] 

        ## pre_compliant = d['num_compliant_used']
        ## pre_compliant_dimen = d['compliant_dimen']  
        ## pre_compliant_pos   = d['compliant_position']  
        ## stiff_ls = d['stiffness_value']  

        ## pre_moveable = d['num_move_used']  
        ## pre_moveable_dimen = d['moveable_dimen']  
        ## pre_moveable_pos = d['moveable_position']  
        ## #d['moveable_max_force'] = [opt.sliding_max_force]*opt.sliding
        
        ## #d['class_type'] = ctype
        ## pre_total_num = d['num_total']  
        ## #d['goal'] = goal.A1.tolist()
        
        ## pre_fixed_dimen += fixed_dimen
        ## pre_fixed_pos   += fixed_pos
        
        ## pre_compliant_dimen += compliant_dimen
        ## pre_compliant_pos   += compliant_pos
        
        ## pre_moveable_dimen  += moveable_dimen
        ## pre_moveable_pos    += sliding_pos
        
        ## dd = {}
        d['num_fixed_used'] += opt.fixed
        d['fixed_dimen']    += fixed_dimen
        d['fixed_position'] += fixed_pos
        d['fixed_ctype']    += fixed_ctype #[ctype] * opt.fixed
        
        d['num_compliant_used'] += opt.compliant
        d['compliant_dimen']    += compliant_dimen
        d['compliant_position'] += compliant_pos
        d['compliant_ctype']    += compliant_ctype #[ctype] * opt.compliant        
        d['stiffness_value']    += stiff_ls

        d['num_move_used']      += opt.sliding
        d['moveable_dimen']     += moveable_dimen
        d['moveable_position']  += sliding_pos
        d['moveable_ctype']     += moveable_ctype #[ctype] * opt.sliding
        d['moveable_max_force'] += [opt.sliding_max_force]*opt.sliding
        
        d['num_total']  += total_num
        #d['goal']       = d['goal']
        #d['start_positoin'] += gate_pos
            
        ut.save_pickle(d, 'reach_problem_dict.pkl')
                       
    else:
        if opt.gps:
            d = dict_from_param_server()
            ut.save_pickle(d, 'reach_problem_dict.pkl')
        else:
            d = {}
            d['num_fixed_used'] = opt.fixed                
            d['fixed_dimen']    = fixed_dimen
            d['fixed_position'] = fixed_pos
            d['fixed_ctype']    = fixed_ctype #[ctype] * opt.fixed

            d['num_compliant_used'] = opt.compliant
            d['compliant_dimen']    = compliant_dimen
            d['compliant_position'] = compliant_pos
            d['compliant_ctype']    = compliant_ctype #[ctype] * opt.compliant
            d['stiffness_value']    = stiff_ls

            d['num_move_used']      = opt.sliding
            d['moveable_dimen']     = moveable_dimen
            d['moveable_position']  = sliding_pos
            d['moveable_ctype']     = moveable_ctype #[ctype] * opt.sliding
            d['moveable_max_force'] = [opt.sliding_max_force]*opt.sliding

            d['num_total'] = total_num
            ## d['goal'] = goal.A1.tolist()
            d['gate_position'] = gate_pos
            
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



