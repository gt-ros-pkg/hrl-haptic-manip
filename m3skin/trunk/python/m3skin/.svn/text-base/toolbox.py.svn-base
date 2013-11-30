#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.

import yaml
import os 
from m3.toolbox import *
from m3.unit_conversion import *




				
def save_data_to_yaml(data, prompt=False):
        print 'Save data [y]?'
        if not m3t.get_yes_no('y'):
                return
        dn=''
        if prompt:
                print 'Enter data name'
                dn=m3t.get_string()
                fn='data/'+dn+'.yml'
        try:
                f=file(fn,'r')
                raw_data= yaml.safe_load(f.read())
                f.close()
        except (IOError, EOFError):
                raw_data={}
                pass
        f=file(fn,'w')
        for k in range(len(data)): # Change to save data. 
                raw_data[k]=data[k]
        print 'Saving...',fn
        f.write(yaml.safe_dump(data, default_flow_style=False,width=200))
        f.close()