#!/usr/bin/env python

import xml.etree.cElementTree as ET

def origin_tag(x,y,z,roll,pitch,yaw):
    return ET.Element('origin',
                      xyz=' '.join((str(x),str(y),str(z))),
                      rpy=' '.join((str(roll),str(pitch),str(yaw))))

def mass_tag(mass):
    return ET.Element('mass', value=str(mass))

def inertia_tag(i):
    print "WARNING: Inertia tag not using robot definition"
    return ET.Element('inertia', ixx='1.0', ixy='0.0', ixz='0.0',
                                 iyy='1.0', iyz='0.0', izz='1.0')

def inertial_tag(i):
    it = ET.Element('inertial')
    it.append(origin_tag(0,0,0,0,0,0))
    #TODO: use robot definition for inertial tag
    print "WARNING: Inertial origin not using robot definition"
    it.append(mass_tag(d_robot.bod_mass[i]))
    it.append(inertia_tag(i))
    return it

def cylinder_tag(i):
    dims = d_robot.bod_dimensions[i]
    r = str(dims[0])
    l = str(dims[1])
    return ET.Element('cylinder', radius=r, length=l)

def sphere_tag(i):
    r = d_robot.bod_dimensions[i][0]
    return ET.Element('sphere', radius=str(r))

def mesh_tag(i):
    print "WARNING: Using Mesh tag.  Scale not implemented"
    #TODO: Implement scale
    filename = d_robot.bod_dimensions[i][0]
    return ET.Element('mesh', filename=filename)

def box_tag(i):
    dims = d_robot.bod_dimensions[i]
    x = str(dims[0])
    y = str(dims[1])
    z = str(dims[2])
    return ET.Element('box', size=' '.join((x,y,z))) 

def geometry_tag(i):
    gt = ET.Element('geometry')
    shape = d_robot.bod_shapes[i]
    if shape == 'capsule':
        print "WARNING: Cannot handle capsule shape accurately, using box instead"
        shape_el = box_tag(i)
    if shape == 'cube':
        shape_el = box_tag(i)
    if shape == 'cylinder':
        shape_el = cylinder_tag(i)
    if shape == 'sphere':
        shape_el = sphere_tag(i)
    if shape == 'mesh':
        shape_el = mesh_tag(i)
    gt.append(shape_el)
    return gt

def color_tag(i):
    colors = d_robot.bod_color[i]
    r = str(colors[0])
    g = str(colors[1])
    b = str(colors[2])
    a = str(colors[3])
    return ET.Element('color', rgba=' '.join((r,g,b,a)))

def material_tag(i):
    mt = ET.Element('material', name="_".join(('material',str(i))))
    mt.append(color_tag(i))
    return mt

def visual_tag(i):
    vt = ET.Element('visual')
    vt.append(origin_tag(0,0,0,0,0,0))
    vt.append(geometry_tag(i))
    vt.append(material_tag(i))
    return vt

def collision_tag(i):
    ct = ET.Element('collision')
    ct.append(origin_tag(0,0,0,0,0,0))
    ct.append(geometry_tag(i))
    return ct

def create_link(i):
    link = ET.Element('link', name=''.join((d_robot.bod_names[i],'_link')))
    link.append(inertial_tag(i))
    link.append(visual_tag(i))
    link.append(collision_tag(i))
    return link

def parent_tag(link_name):
    return ET.Element('parent', link=link_name)

def child_tag(link_name):
    return ET.Element('child', link=link_name)

def joint_connection(i):
    conn = d_robot.b_jt_attach[i]
    parent_num = conn[1]
    child_num = conn[0]
    if parent_num < 0.:
        par_tag = parent_tag('root_link')
    else:
        par_tag = parent_tag(''.join((d_robot.bod_names[parent_num],'_link')))
    ch_tag = child_tag(''.join((d_robot.bod_names[child_num], '_link')))
    return par_tag, ch_tag

def axis_tag(i):
    axes = d_robot.b_jt_axis[i]
    x = str(axes[0])
    y = str(axes[1])
    z = str(axes[2])
    return ET.Element('axis', xyz=' '.join((x,y,z)))

def limit_tag(i):
    min_ = d_robot.b_jt_limits_min[i]
    max_ = d_robot.b_jt_limits_max[i]
    effort = 1000.0
    velocity = 1000.0
    return ET.Element('limit', lower=str(min_), upper=str(max_),
                              effort=str(effort), velocity=str(velocity))

def create_joint(i):
    joint = ET.Element('joint', name=''.join((d_robot.bod_names[i],'_joint')),
                                type='revolute')
    trans = d_robot.b_jt_anchor[i]
    joint.append(origin_tag(trans[0], trans[1], trans[2], 0, 0, 0))
    parent_tag, child_tag = joint_connection(i)
    joint.append(parent_tag)
    joint.append(child_tag)
    joint.append(axis_tag(i))
    joint.append(limit_tag(i))
    return joint

def root_link():
    return ET.Element('link', name='root_link')

def robot_description_to_urdf(d_robot, filename):
    tree = ET.Element('robot', name=filename)
    tree.append(root_link())
    for i in xrange(len(d_robot.bod_names)):
        tree.append(create_joint(i))
        tree.append(create_link(i))
    return ET.ElementTree(tree)

if __name__=='__main__':
    import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
    import hrl_common_code_darpa_m3.robot_config.multi_link_seven_planar as d_robot
    tree = robot_description_to_urdf(d_robot, 'six_link_planar')
    with open('urdf_test.xml','wb') as f:
       tree.write(f)
