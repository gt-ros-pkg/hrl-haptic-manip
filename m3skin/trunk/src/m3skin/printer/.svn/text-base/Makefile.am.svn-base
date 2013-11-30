# M3 -- Meka Robotics Real-Time Control System
# Copyright (c) 2010 Meka Robotics
# Author: edsinger@mekabot.com (Aaron Edsinger)
# 
# M3 is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# M3 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with M3.  If not, see <http://www.gnu.org/licenses/>.

m3lcj_dir = $(top_srcdir)/src
bin_PROGRAMS = m3skin_printer
m3skin_printer_SOURCES = m3skin_printer.c 
m3skin_printer_LDADD = -lpthread

AM_CPPFLAGS = -I/opt/etherlab/include  \
				-I/usr/local/src/ethercat/master\
				-I/usr/realtime/include \
				-I$(m3lcj_dir)\
				-D__IN_RTAI__ -ffast-math 







