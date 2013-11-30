

import serial


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


if __name__ == '__main__':

    dev_name = '/dev/ttyUSB0'
    baudrate = 115200

    serial_dev = setup_serial(dev_name, baudrate)

    while True:
        l = serial_dev.readline()
        print 'l:', l


