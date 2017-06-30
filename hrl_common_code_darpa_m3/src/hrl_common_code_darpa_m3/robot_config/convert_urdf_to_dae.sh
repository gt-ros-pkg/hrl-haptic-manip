#!/bin/bash -x

# urdf -> dae
rosrun collada_urdf urdf_to_collada three_link_planar_capsule.urdf three_link_planar_capsule.dae

# dae -> rounded dae 
rosrun moveit_ikfast round_collada_numbers.py three_link_planar_capsule.dae three_link_planar_capsule.rounded.dae 5

# print info
/usr/bin/openrave-robot.py three_link_planar_capsule.dae --info links

# xml ->

