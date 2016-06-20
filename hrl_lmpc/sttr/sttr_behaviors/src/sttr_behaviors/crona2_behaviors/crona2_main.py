#!/usr/bin/env python
#######################################################
# 
# cRoNA2 Main script to perform actions
# 
########################################################
# meant to be an interface that allows you to perform the different crona2 behaviors
# rviz interface is pretty much doing just this, so may not be used anymore

import roslib
roslib.load_manifest('sttr_behaviors')
from sttr_behaviors.crona2_behaviors import crona2_lift
from sttr_behaviors.crona2_behaviors import crona2_lower_down
from sttr_behaviors.crona2_behaviors import crona2_return_home
from sttr_behaviors.crona2_behaviors import crona2_move_towards_object
from sttr_behaviors.crona2_behaviors import crona2_roll
#from sttr_behaviors.crona2_behaviors import crona2_manipulate

class cRoNA2Main():
    def __init__(self):
	self.lift = crona2_lift.cRoNA2Lift()		
	self.lower_down = crona2_lower_down.cRoNA2LowerDown()	
	self.return_home = crona2_return_home.cRoNA2ReturnHome()	
	self.move_towards_object = crona2_move_towards_object.cRoNA2MoveTowardsObject()	
	self.roll = crona2_roll.cRoNA2Roll()	
	#self.manipulate = crona2_manipulate.cRoNA2Manipulate()	

    def start(self):
	self.return_home.start()
	self.lower_down.start()
	self.move_towards_object.start()
	self.lift.start()

if __name__ == '__main__':

    c2m = cRoNA2Main()
    raw_input('Press Enter to start')
    c2m.start()
