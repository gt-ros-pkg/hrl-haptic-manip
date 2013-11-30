import ode
import roslib
roslib.load_manifest('visualization_msgs')
from visualization_msgs.msg import Marker

class RobSimMaker:
    def __init__(self, world, space):
        self.world = world
        self.space = space

    def create_body(self, shape, dim, com_pos, color, mass, density, rotation, inertia, name = None):
        body = ode.Body(self.world)
        body.dim = dim
        body.com_pos = com_pos
        body.color = color
        body.shape = shape
        body.name = name
        M = ode.Mass()
        if not mass == None:
            print "specific mass not implemented yet, use density option"
        elif not density == None:
            if shape == 'cylinder':
                body.rad = dim[0]/2.0
                body.len = dim[2]
                body.vis = Marker.CYLINDER
                M.setCappedCylinder(density, 3, body.rad, body.len)
            elif shape == 'cube':
                body.vis = Marker.CUBE
                M.setBox(density, dim[0], dim[1], dim[2])
        elif density == None and mass == None:
            print "you didn't specify a mass  or density"
            return None
        body.setMass(M)
        body.setPosition(com_pos)
        if not rotation == None:
            body.setRotation(rotation)
        return body

    def create_geom(self, body, name, dim=None, shape=None, pos=None):
        if not body == None:
            if body.shape == 'cylinder':
                geom = ode.GeomCCylinder(self.space, body.rad, body.len)
            elif body.shape == 'cube':
                geom = ode.GeomBox(self.space, body.dim)
            geom.setBody(body)
        elif not shape == None:
            if shape == 'cylinder':
                geom = ode.GeomCCylinder(self.space, dim[0]/2, dim[2])
            elif shape == 'cube':
                geom = ode.GeomBox(self.space, dim)
            geom.setPosition(pos)
        geom.name = name
        return geom

    def create_joint(self, bodies, jt_type, anchor, axis, vis_size, vis_color, feedback, jt_lim = None):
        if jt_type == 'revolute':
            jt = ode.HingeJoint(self.world)
        elif jt_type == 'fixed':
            jt = ode.FixedJoint(self.world)
        elif jt_type == 'prismatic':
            jt = ode.SliderJoint(self.world)
        elif jt_type == 'planar':
            jt = ode.Plane2DJoint(self.world)
        else:
            print 'not even aware of this type yet, check documentation of pyode and implement yourself'
        if not (jt_type == 'planar'):
            jt.attach(bodies[0], bodies[1])
        else:
            jt.attach(bodies[0], bodies[1], 0)
        if not (jt_type == 'fixed' or jt_type == 'prismatic'):
            jt.setAnchor(anchor)
            jt.setAxis(axis)
        elif jt_type == 'fixed':
            jt.setFixed()
        elif jt_type == 'prismatic':
            jt.setAxis(axis)
        jt.size = vis_size
        jt.color = vis_color
        jt.jt_type = jt_type
        if jt_lim != None:
            jt.setParam(ode.ParamLoStop, jt_lim[0])
            jt.setParam(ode.ParamHiStop, jt_lim[1])
        if feedback == True:
            jt.setFeedback(flag=True)
        return jt

        
