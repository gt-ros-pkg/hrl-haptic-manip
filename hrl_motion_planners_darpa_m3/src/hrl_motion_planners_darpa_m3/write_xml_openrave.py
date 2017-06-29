
#
# helper functions to write an XML file that OpenRAVE can parse.
#


import os 
import math 


#convert list to string separated by sep (e.g ' ', or ',')
def list_str(l, sep=' '):
    return ' '.join(map(lambda x: str(x), l))

# translation, rotationaxis, diffuse_color are lists.
def write_openrave_cylinder_KinBody(file_object, name, translation,
                        rotationaxis, radius, height, diffuse_color):
    translation = list_str(translation)
    rotationaxis = list_str(rotationaxis)
    diffuse_color = list_str(diffuse_color)
    radius = str(radius)
    height = str(height)
    file_object.write('\t <KinBody name="'+name+'"> \n'+
                '\t \t <translation> '+translation+' </translation> \n'+
                '\t \t <Body type="dynamic"> \n'+
                '\t \t \t <Geom type="cylinder"> \n'+
                '\t \t \t \t <rotationaxis> ' + rotationaxis + ' </rotationaxis> \n'+
                '\t \t \t \t <radius> ' + radius + ' </radius> \n'+
                '\t \t \t \t <height> ' + height + ' </height> \n'+
                '\t \t \t \t <diffuseColor> ' + diffuse_color + ' </diffuseColor> \n'+
                '\t \t \t </Geom> \n'+
                '\t \t </Body> \n'+
                '\t </KinBody>\n\n')

# translation, rotationaxis, diffuse_color are lists.
def write_openrave_box_KinBody(file_object, name, translation,
                               extent, diffuse_color):
    translation = list_str(translation)
    extent = list_str(extent)
    diffuse_color = list_str(diffuse_color)
    file_object.write('\t <KinBody name="'+name+'"> \n'+
                '\t \t <translation> '+translation+' </translation> \n'+
                '\t \t <Body type="dynamic"> \n'+
                '\t \t \t <Geom type="box"> \n'+
                '\t \t \t \t <Extents>' + extent + '</Extents> \n'+
                '\t \t \t \t <diffuseColor> ' + diffuse_color + ' </diffuseColor> \n'+
                '\t \t \t </Geom> \n'+
                '\t \t </Body> \n'+
                '\t </KinBody>\n\n')

def environment_start(file_object):
    file_object.write('<Environment> \n\n')

def environment_end(file_object):
    file_object.write('</Environment> \n\n')

def write_robot(file_object, robot_file, effector, base,
                  translation, rotationaxis):
    translation = list_str(translation)
    rotationaxis = list_str(rotationaxis)
    file_object.write('\t <Robot file="'+robot_file+'"> \n' +
                '\t \t <Manipulator> \n' +
                '\t \t \t <effector> ' + effector + '</effector> \n'+
                '\t \t \t <base> ' + base + '</base> \n'+
                '\t\t\t <!-- grasp goal with respect to the effector-->\n' +
                '\t \t \t <Translation> ' + translation + '</Translation> \n'+
                '\t \t \t <RotationAxis> ' + rotationaxis + '</RotationAxis> \n'+
                '\t \t </Manipulator> \n'+
                '\t </Robot> \n\n')

def write_linkage_xml_file(b_jts, bodies, fname):
    f_xml = open(fname, 'w')
    f_xml.write('<!--THIS FILE IS AUTO-GENERATED, please do not mess with it directly -->\n'+
                '<!--modify the file referred to in planar_openrave.py instead -->\n'+
                '<Robot name="3DOFRobot"> \n'+
                '\t<KinBody> \n'+
                '\t\t<!-- Create the base body, it should never move--> \n'+
                '\t\t<!-- Note that all translations and rotations are with respect to this base-->\n'+
                '\t\t<!-- For example, the robot at the identity transformation is equivalent to the identity transformation of the first body.-->\n'+
                '\t\t<Body name="Base" type="dynamic"> \n'+
                '\t\t\t<Translation>0.0  0.0  0.0</Translation> \n'+
                '\t\t\t<Geom type="cylinder"> \n'+
                '\t\t\t\t<rotationaxis>1 0 0 90</rotationaxis> \n'+
                '\t\t\t\t<radius>0.03</radius> \n'+
                '\t\t\t\t<height>0.04</height> \n'+
                '\t\t\t\t<diffuseColor>0 0 1</diffuseColor> \n'+
                '\t\t\t</Geom> \n'+
                '\t\t</Body> \n')

    for i in xrange(bodies['num_links']):
        if i == 0:
            name_prev = "Base"
            trans1 = '0 0 0'
        else:
            name_prev = bodies['name'][i-1]
            trans1 = str(b_jts['anchor'][i][0]-b_jts['anchor'][i-1][0])+' '+str(b_jts['anchor'][i][1]-b_jts['anchor'][i-1][1])+' '+str(b_jts['anchor'][i][2]-b_jts['anchor'][i-1][2])

        trans2 = '0 '+str(-bodies['dim'][i][2]/2.0)+' 0'

        trans_sphere = '0 '+str(-bodies['dim'][i][2])+' 0'
        radius = str(bodies['dim'][i][1]/2.0)

        extent = str(bodies['dim'][i][1]/2.0)+' '+str(bodies['dim'][i][2]/2.0)+' '+str(bodies['dim'][i][0]/2.0)
        name = bodies['name'][i]
        axis = str(b_jts['axis'][i][0])+' '+str(b_jts['axis'][i][1])+' '+str(b_jts['axis'][i][2])

        jt_limits = str(math.degrees(b_jts['jt_lim_min'][i]))+' '+str(math.degrees(b_jts['jt_lim_max'][i]))
        color = str(bodies['color'][i][0])+' '+str(bodies['color'][i][1])+' '+str(bodies['color'][i][2])
        f_xml.write('\t\t<!-- the second movable link--> \n'+
                    '\t\t<Body name="'+name+'" type="dynamic"> \n'+
                    '\t\t\t<offsetfrom>'+name_prev+'</offsetfrom> \n'+
                    '\t\t\t<Translation>'+trans1+'</Translation> \n'+

                    '\t\t\t<Geom type="box"> \n'+
                    '\t\t\t\t<Translation>'+trans2+'</Translation> \n'+
                    '\t\t\t\t<Extents>'+extent+'</Extents> \n'+
                    '\t\t\t\t<diffuseColor>'+color+'</diffuseColor> \n'+
                    '\t\t\t</Geom> \n'+

                    '\t\t\t<Geom type="sphere"> \n'+
                    '\t\t\t\t<Translation>'+trans_sphere+'</Translation> \n'+
                    '\t\t\t\t<Radius>'+radius+'</Radius> \n'+
                    '\t\t\t\t<diffuseColor>'+color+'</diffuseColor> \n'+
                    '\t\t\t</Geom> \n'+

                    '\t\t</Body> \n'+
                    '\t\t<!-- declare a circular hinge joint (circular joints have no limits) --> \n'+
                    '\t\t<Joint circular="false" name="'+name+'" type="hinge"> \n'+
                    '\t\t\t<Body>'+name_prev+'</Body> \n'+
                    '\t\t\t<Body>'+name+'</Body> \n'+
                    '\t\t\t<offsetfrom>'+name+'</offsetfrom> \n'+
                    '\t\t\t<weight>4</weight> \n'+
                    '\t\t\t<limitsdeg>'+jt_limits+'</limitsdeg> \n'+
                    '\t\t\t<axis>'+axis+'</axis> \n'+
                    '\t\t\t<maxvel>4</maxvel> \n'+
                    '\t\t\t<resolution>1</resolution> \n'+
                    '\t\t</Joint> \n\n')

    trans = '0 '+str(-1*bodies['dim'][-1][2])+' 0'
    f_xml.write('\t\t<!-- set the transparency of every geometry in the KinBody--> \n'+
                '\t\t<transparency>0.01</transparency> \n'+
                '\t</KinBody> \n\n\n'+
                
                '\t<!-- Specifying the manipulator structure--> \n'+
                '\t<Manipulator name="arm"> \n'+
                '\t\t<effector>'+bodies['name'][-1]+'</effector>   <!-- last link where end effector is attached--> \n'+
                '\t\t<base>Base</base>           <!-- base link--> \n'+
                '\t\t<!-- grasp goal with respect to the effector--> \n'+
                '\t\t<Translation>'+trans+'</Translation> \n'+
                '\t\t<RotationAxis>1 0 0 0</RotationAxis> \n'+
                '\t</Manipulator> \n'+
                '</Robot>')
    f_xml.close()
    print "wrote new sim.robot.xml file"

def write_environment_xml_file(d, fname, robot_xml):
    f_xml = open(fname, 'w')
    
    f_xml.write('<Environment> \n'+
                '\t <Robot name="3DOFRobot" file="'+robot_xml+'"> \n'+
                '\t </Robot> \n')

    for i in xrange(d['num_move_used']):
        trans = [2.0, 0., 0.0]
        name = "movable"+str(i+1).zfill(2)
        rotation = [1, 0, 0, 90]
        diffuse_color = [1, 1, 1]
        radius = d['moveable_dimen'][i][0]
        height = d['moveable_dimen'][i][2]
        write_openrave_cylinder_KinBody(f_xml, name, trans, rotation,
                                        radius, height, diffuse_color)
            

    for i in xrange(d['num_fixed_used']):        
        trans = [0.8, 0.6, 0.0]
        name = "fixed"+str(i+1).zfill(2)
        rotation = [1, 0, 0, 90]
        radius = d['fixed_dimen'][i][0]
        height = d['fixed_dimen'][i][2]
        diffuse_color = [1, 0.2, 0.2]
        write_openrave_cylinder_KinBody(f_xml, name, trans, rotation,
                                        radius, height, diffuse_color)
            
    trans = [0.9, 0.3, 0.05]
    name = 'target'
    rotation = [1, 0, 0, 90]
    radius = 0.02
    height = 0.01
    diffuse_color = [0, 1, 0]
    write_openrave_cylinder_KinBody(f_xml, name, trans, rotation,
                                    radius, height, diffuse_color)

    f_xml.write('</Environment>')
    f_xml.close()
    print "wrote new planar.environment.xml file"

def write_cuboid_obstacles(file_obj, n_obs, init_x, extent):
    obstacle_nm_l = ['obstacle'+str(i) for i in range(1,n_obs+1)]

    # initial position of obstacles
    pos_list = [[(i+1) * init_x, 0. , 0.] for i in range(n_obs)]
    diffuse_color = [1, 0.2, 0.2]

    for i in range(n_obs):
        nm = obstacle_nm_l[i]
        trans = pos_list[i]
        write_openrave_box_KinBody(file_obj, nm, trans, extent,
                                   diffuse_color)


