
#
# Publish ADC data over ROS.
#
# One example of visualizing the data:
# rxplot /arduino/ADC/data[0] -b 15 -r100 -p5
# rxplot /arduino/ADC/data[0] /arduino/ADC/data[1] /arduino/ADC/data[2] /arduino/ADC/data[3] -b 15 -r100 -p5
#
#


import serial

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy
from hrl_msgs.msg import FloatArray


def setup_serial(dev_name, baudrate):
    try:
        serial_dev = serial.Serial(dev_name, timeout=1.)
        if(serial_dev == None):
            raise RuntimeError("robotis_servo: Serial port not found!\n")

        serial_dev.setBaudrate(baudrate)
        serial_dev.setParity('N')
        serial_dev.setStopbits(1)
        serial_dev.open()

        serial_dev.flushOutput()
        serial_dev.flushInput()
        return serial_dev

    except (serial.serialutil.SerialException), e:
        raise RuntimeError("robotis_servo: Serial port not found!\n")

def get_adc_data(serial_dev):
    ln = serial_dev.readline()
    return map(int, ln.split(','))


if __name__ == '__main__':

    dev_name = '/dev/ttyUSB0'
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
        fa.data = get_adc_data(serial_dev)
        pub.publish(fa)

    serial_dev.close()


