#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Read the throttle and steering setpoints coming from the Arduino controller over serial port.
"""

import os

import serial
import roslib
import rospy
from std_msgs.msg import Float32

import helpers

roslib.load_manifest('arduino_controller')

# region Constants

STEERING_MIN = int(os.environ.get('STEERING_MIN', '1000'))
STEERING_MAX = int(os.environ.get('STEERING_MAX', '1984'))
STEERING_CENTER = int(os.environ.get('STEERING_CENTER', '1496'))

THROTTLE_MIN = int(os.environ.get('THROTTLE_MIN', '1040'))
THROTTLE_MAX = int(os.environ.get('THROTTLE_MAX', '1996'))
THROTTLE_CENTER = int(os.environ.get('THROTTLE_CENTER', '1532'))

# endregion

# region Serial port

SERIAL_PORT = os.environ.get('SERIAL_PORT', '/dev/ttyACM0')
SERIAL_SPEED_BAUDS = int(os.environ.get('SERIAL_SPEED_BAUDS', '57600'))

ser = serial.Serial(SERIAL_PORT, SERIAL_SPEED_BAUDS)
rospy.loginfo('Connected to serial port %s at %d bauds.', SERIAL_PORT, SERIAL_SPEED_BAUDS)

# endregion


def controller():
    publisher_throttle = rospy.Publisher('throttle/normalized', Float32, queue_size=1)
    publisher_steering = rospy.Publisher('steering/normalized', Float32, queue_size=1)

    rospy.init_node('arduino_controller', anonymous=True)

    rospy.loginfo('Reading data from serial port. Press CTRL+C to exit.')
    while not rospy.is_shutdown():

        # Read serial data
        data_byte_array = ser.readline()

        # Extract steering and throttle setpoints from serial data
        data_dict = helpers.serial_data_to_dict(data_byte_array)
        try:
            steering = data_dict['CH1']
            throttle = data_dict['CH2']
        except KeyError as e:
            # `CH1` or `CH2` are not in `data_dict`
            rospy.logerr('Could not find key [%s] in data_dict.', e.args[0])
            steering = STEERING_CENTER
            throttle = THROTTLE_CENTER

        rospy.logdebug('Steering = %d, Throttle = %d', steering, throttle)

        # Normalize steering and throttle setpoints to [-1.0; +1.0]
        steering_normalized = helpers.microseconds_to_normalized(steering,
                                                                 min_reading=STEERING_MIN,
                                                                 center_reading=STEERING_CENTER,
                                                                 max_reading=STEERING_MAX)
        throttle_normalized = helpers.microseconds_to_normalized(throttle,
                                                                 min_reading=THROTTLE_MIN,
                                                                 center_reading=THROTTLE_CENTER,
                                                                 max_reading=THROTTLE_MAX)

        rospy.logdebug('Steering normalized = %d, Throttle normalized = %d',
                       steering_normalized, throttle_normalized)

        publisher_throttle.publish(Float32(throttle_normalized))
        publisher_steering.publish(Float32(steering_normalized))


if __name__ == '__main__':

    try:
        controller()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down...')
