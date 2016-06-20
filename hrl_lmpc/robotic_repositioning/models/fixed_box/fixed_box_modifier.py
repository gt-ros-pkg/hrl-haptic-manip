#!/usr/bin/env python

import sys
import re
import math

f = open('model.sdf','rw')
nf = open('model_mod.sdf','w')

lines = f.readlines()
prev_line = ''
prev_prev_line = ''
for line in lines:
  line = line.rstrip()
  if "<box>" in prev_line and "<size>" in line:
    num_list = re.findall(r"[-+]?\d*\.\d+|\d+",line)
    m = re.match('(\s+)',line)
    indent = len(m.group(0))
    new_line = ''
    for i in range(indent):
      new_line += ' '
    if "size" in line:
      new_line += "<size>%f %f %f</size>" %(float(num_list[0]), float(num_list[1])+2.*0.3901*math.sin(math.radians(float(sys.argv[1]))), float(num_list[2]))
    nf.write(new_line+'\n')
  else:
    nf.write(line+'\n')
  prev_prev_line = prev_line
  prev_line = line

    
