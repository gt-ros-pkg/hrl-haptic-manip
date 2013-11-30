#!/usr/bin/python

from threading import Thread
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.viz as m3v
import m3.humanoid
import subprocess
import time

class M3SkinViz(Thread):
    def __init__ (self, stride_ms=100):
        Thread.__init__(self)
        self.stride_ms = stride_ms
        self.proxy = m3p.M3RtProxy()
        self.proxy.start(True, True)
        bot_name = m3t.get_robot_name() 
        bot = m3.humanoid.M3Humanoid(bot_name)
        self.proxy.subscribe_status(bot)
        self.viz = m3v.M3Viz(self.proxy,bot)        
        self.done = False

    def run(self):
	while not self.done:	
            self.proxy.step()        
            self.viz.step()
            time.sleep(self.stride_ms/1000.0)
        self.viz.stop()
        self.proxy.stop()
            
    def stop_viz(self):
        self.done = True

if __name__ == '__main__':
    viz = M3SkinViz()
    viz.start()

    dev_null = open('/dev/null', 'w')
    p = subprocess.Popen(['roslaunch', 'm3skin_rviz_demo', 'm3skin_rviz_demo.launch'], stdout=dev_null, stderr=dev_null)
    dev_null.close()

    print '------------------------------'
    print 'RVIZ loaded.  Hit <Q> to exit.'
    k = 'a'
    while not (k == 'q' or k =='Q'):
        k=m3t.get_keystroke()

    viz.stop_viz()
    


