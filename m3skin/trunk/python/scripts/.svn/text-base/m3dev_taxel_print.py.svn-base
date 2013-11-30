#! /usr/bin/python

#Copyright  2008, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted. 


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.

import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
#import m3skin.component_factory as m3f
import math
import m3skin.taxel_array_ec as mta

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()		
		self.components = []
		self.comp = None

	def stop(self):
		self.proxy.stop()
	def start(self):
		self.proxy.start()
		
		self.comp = mta.M3TaxelArrayEc('m3taxel_array_ec01')
		
		self.proxy.subscribe_status(self.comp)
		#self.proxy.publish_command(self.comp) 
		#self.proxy.publish_param(self.comp)	
		self.proxy.make_operational_all()		
		self.proxy.step()		
			
	def step(self):
		print '---------------- debug=', self.comp.status.debug
		div = 8
		#for i in range(192):
		for i in range(384/div):		
			#print i, self.comp.status.taxel_value[i]
			print i*div, self.comp.status.taxel_value[i*div], '--', i*div+1, self.comp.status.taxel_value[i*div+1], '--',i*div+2, self.comp.status.taxel_value[i*div+2], '--', i*div+3, self.comp.status.taxel_value[i*div+3], '--',i*div+4, self.comp.status.taxel_value[i*div+4], '--', i*div+5, self.comp.status.taxel_value[i*div+5], '--',i*div+6, self.comp.status.taxel_value[i*div+6], '--', i*div+7, self.comp.status.taxel_value[i*div+7]
			#if i != self.comp.status.taxel_value[i]:
			#	print "ERROR", self.comp.status.taxel_value[i]
		
		self.proxy.step()

if __name__ == '__main__':
        print "Starting..."
	t=M3Proc()
        t.start()
        i = 0
	try:
		while True:
			time.sleep(0.1)
	                t.step()
	except (KeyboardInterrupt,EOFError):
		pass
        print "Stopping..."
	t.stop()
        print "Stopped."


