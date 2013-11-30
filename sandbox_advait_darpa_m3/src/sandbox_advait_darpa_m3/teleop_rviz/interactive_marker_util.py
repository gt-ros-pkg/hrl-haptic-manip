
#
# Code copied from basic_controls.py and then modified.
#


"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import math, numpy as np

import roslib; roslib.load_manifest('interactive_markers')
import rospy

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl


# scale - float
# color - (r,g,b,a)
# mtype - 'cube', 'sphere'
def make_marker(scale, color, mtype):
    ss = 0.3
    marker = Marker()
    if mtype == 'cube':
        marker.type = Marker.CUBE
        ss = ss * 1/(math.pow(3, 1./3))
    elif mtype == 'sphere':
        marker.type = Marker.SPHERE
    else:
        raise RuntimeError('Undefined marker type')

    marker.scale.x = ss * scale
    marker.scale.y = ss * scale
    marker.scale.z = ss * scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    return marker

def make_3dof_marker_position(ps, scale, color, mtype):
    return make_marker_flexible(True, ps, scale, color, mtype,
                                ignore_rotation = True)

def make_marker_position_xy(ps, scale, color, mtype):
    return make_marker_flexible(True, ps, scale, color, mtype,
                                ignore_rotation=True, ignore_z=True)

# initial_position - 3x1 np matrix
# pose - geometry_msgs/PointStamped
# scale, color, mtype -- see make_marker.
def make_6dof_marker(fixed, ps, scale, color, mtype):
    return make_marker_flexible(fixed, ps, scale, color, mtype,
                                ignore_rotation = False)

def make_marker_flexible(fixed, ps, scale, color, mtype,
                         ignore_rotation, ignore_x=False,
                         ignore_y=False, ignore_z=False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = ps.header.frame_id
    int_marker.pose.position.x = ps.point.x
    int_marker.pose.position.y = ps.point.y
    int_marker.pose.position.z = ps.point.z
    int_marker.scale = scale

    int_marker.name = 'simple_6dof'
    int_marker.description = ''

    # insert a marker
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(make_marker(scale, color, mtype))
    int_marker.controls.append(control)

    if fixed:
        int_marker.name += '_fixed'
        int_marker.description += '\n(fixed orientation)'

    if not ignore_x:
        control = InteractiveMarkerControl()
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    if not ignore_y:
        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.orientation.w = 1
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    if not ignore_z:
        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    if not ignore_rotation:
        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'rotate_x'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.orientation.w = 1
        control.name = 'rotate_y'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    return int_marker


# add this menu handler to the Interactive Marker.
# server - InteractiveMarkerServer
def add_menu_handler(int_marker, menu_handler, server):
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    int_marker.controls.append(control)
    menu_handler.apply(server, int_marker.name)
    server.applyChanges()



