# MEKA CONFIDENTIAL
# 
# Copyright 2011 
# Meka Robotics LLC
# All Rights Reserved.
# 
# NOTICE:  All information contained herein is, and remains
# the property of Meka Robotics LLC. The intellectual and 
# technical concepts contained herein are proprietary to 
# Meka Robotics LLC and may be covered by U.S. and Foreign Patents,
# patents in process, and are protected by trade secret or copyright law.
# Dissemination of this information or reproduction of this material
# is strictly forbidden unless prior written permission is obtained
# from Meka Robotics LLC.

## Process this file with automake to produce Makefile.in

#//////////////////////////////////////////////////////////////////

m3skin_dir = $(top_srcdir)/src
m3rt_dir = $(prefix)/include/m3rt
m3rtb_dir = $(prefix)/include/m3rt/base
rtai_inc = /usr/realtime/include 
ros_dir = /opt/ros/cturtle/ros
m3_dir = $(top_srcdir)/../m3/src
eigen_dir = $(top_srcdir)/../eigen

libm3skincomponentsincludedir = $(includedir)/m3skin/components

libm3skincomponentsinclude_HEADERS = \
	taxel_array_ec.h\
	taxel_array_ec.pb.h\
	taxel_array.h\
	taxel_array.pb.h\
	taxel_array_virtual.h
	
clean-local:
	rm -f *.pb.*
	
#////////////////////////// M3 SYS ////////////////////////////
taxel_array_ec.pb.cpp: taxel_array_ec.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ taxel_array_ec.proto
	mv taxel_array_ec.pb.cc taxel_array_ec.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3skin taxel_array_ec.proto

taxel_array.pb.cpp: taxel_array.proto
	protoc -I$(prefix)/include/m3rt/base -I./ --cpp_out=./ taxel_array.proto
	mv taxel_array.pb.cc taxel_array.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I./ --python_out=$(top_srcdir)/python/m3skin taxel_array.proto
	
proto: taxel_array_ec.pb.cpp taxel_array.pb.cpp

proto_install: 
	if test -d $(includedir)/m3skin; then echo '' ; else mkdir $(includedir)/m3skin;fi
	if test -d $(includedir)/m3skin/components; then echo '' ; else mkdir $(includedir)/m3skin/components;fi
	cp *.proto $(includedir)/m3skin/components 

#///////////////////////////////////////////////////////////////////////////////////////////////////
lib_LTLIBRARIES = libm3skincomponents.la 
libm3skincomponents_la_LIBADD = -lprotobuf -lm3toolbox -lyaml-cpp -lm3base -lm3hardware $(EIGEN2_LIBS) -ltf -ltf_conversions

libm3skincomponents_la_LDFLAGS = -version-info 1:1:0 -module \
    -L/usr/realtime/lib \
    -L/opt/ros/cturtle/stacks/geometry/tf/lib \
    -L/opt/ros/cturtle/stacks/geometry/tf_conversions/lib
    
libm3skincomponents_la_CPPFLAGS =  -I$(rtai_inc) -I$(m3skin_dir) -I$(m3_dir) -I$(m3rt_dir) -I$(eigen_dir) -I$(m3rtb_dir)  -ffast-math -pipe \
	-I$(ros_dir)/core/roscpp/include \
	-I$(top_srcdir)/../m3/ros/m3_msgs/srv_gen/cpp/include \
	-I$(top_srcdir)/../m3/ros/m3_msgs/msg_gen/cpp/include \
	-I$(top_srcdir)/ros/m3skin_ros/msg_gen/cpp/include \
        -I$(top_srcdir)/ros/m3skin_ros/srv_gen/cpp/include \
	-I$(ros_dir)/m3_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/core/roslib/include \
	-I$(ros_dir)/core/rosconsole/include \
	-I$(ros_dir)/core/roslib/msg_gen/cpp/include \
	-I/opt/ros/cturtle/stacks/geometry/tf/include \
	-I/opt/ros/cturtle/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include/ \
	-I/opt/ros/cturtle/stacks/geometry/bullet/include/ \
	-I/opt/ros/cturtle/stacks/geometry/tf/msg_gen/cpp/include/ \
	-I/opt/ros/cturtle/stacks/geometry/tf_conversions/include/ \
	-I/opt/ros/cturtle/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include/geometry_msgs/ \
	-I$(ros_dir)/3rdparty/xmlrpcpp/src
	
libm3skincomponents_la_SOURCES = \
	taxel_array_ec.pb.cpp\
 	taxel_array_ec.cpp\
 	taxel_array.pb.cpp\
	factory_proxy.cpp\
 	taxel_array.cpp\
 	taxel_array_virtual.cpp


#////////////////////////////////////////////////////////////////////


