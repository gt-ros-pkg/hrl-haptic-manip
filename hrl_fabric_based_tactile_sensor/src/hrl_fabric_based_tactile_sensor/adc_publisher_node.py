#!/usr/bin/env python

#
# Publish ADC data over ROS.
#
# One example of visualizing the data:
# rxplot /arduino/ADC/data[0] -b 15 -r100 -p5
# rxplot /arduino/ADC/data[0] /arduino/ADC/data[1] /arduino/ADC/data[2] /arduino/ADC/data[3] -b 15 -r100 -p5
#
#

import serial

import rospy
from hrl_msgs.msg import FloatArray


def setup_serial(dev_name, baudrate):
    try:
        serial_dev = serial.Serial(dev_name, timeout=1.)
        if(serial_dev is None):
            raise RuntimeError("[%s]: Serial port %s not found!\n" % (rospy.get_name(), dev_name))

        serial_dev.setBaudrate(baudrate)
        serial_dev.setParity('N')
        serial_dev.setStopbits(1)
        #serial_dev.open()

        serial_dev.flushOutput()
        serial_dev.flushInput()
        return serial_dev

    except serial.serialutil.SerialException as e:
        rospy.logwarn("[%s] Error initializing serial port %s", rospy.get_name(), dev_name)
        return []


def get_adc_data(serial_dev, num_adc_inputs):
    try:
        ln = serial_dev.readline()
        try:
            l = map(int, ln.split(','))
        
        except ValueError:
            rospy.logwarn('[%s] Received suspect data: %s from socket.' % (rospy.get_name(), ln))
            #rospy.logwarn('Something fishy with line read, not sure what ... getting new data')
            serial_dev.flush()
            return []

        if len(l) != num_adc_inputs:
            rospy.logwarn('Number of ADC values does not match prescribed number of inputs. Something fishy with the serial port.')
            serial_dev.flush()
            return []  # get_adc_data(serial_dev, num_adc_inputs)
        return l

    except:
        rospy.logwarn('[%s] Unable to read line. Recommend setup serial again.', rospy.get_name())
        return [-1]
        

    


if __name__ == '__main__':
    dev_name = '/dev/ttyUSB4'
    # dev_name = '/dev/ttyACM0'
    # dev_name = '/dev/robot/arduino1'
    # dev_name = '/dev/robot/arduino2'

    baudrate = 115200

    serial_dev = setup_serial(dev_name, baudrate)

    pub = rospy.Publisher('/arduino/ADC', FloatArray)
    rospy.init_node('adc_publisher_node')

    for i in range(10):
        ln = serial_dev.readline()

    rospy.loginfo('Started publishing ADC data')

    while not rospy.is_shutdown():
        fa = FloatArray()
        fa.header.stamp = rospy.Time.now()
        fa.data = get_adc_data(serial_dev, 6)
        if fa.data == []:
            rospy.logwarn('Data from taxels was emtpy ...')
        else:
            pub.publish(fa)

    serial_dev.close()
