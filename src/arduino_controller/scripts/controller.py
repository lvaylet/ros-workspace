#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Read the throttle and steering setpoints coming from the Arduino controller over serial port.
"""

import os

import serial
import roslib
import rospy
from std_msgs.msg import UInt16, Float32

import helpers

roslib.load_manifest('arduino_controller')

# region Constants

STEERING_MIN = rospy.get_param('~steering_min', 1000)
STEERING_CENTER = rospy.get_param('~steering_center', 1496)
STEERING_MAX = rospy.get_param('~steering_max', 1984)

THROTTLE_MIN = rospy.get_param('~throttle_min', 1040)
THROTTLE_MAX = rospy.get_param('~throttle_max', 1996)
THROTTLE_CENTER = rospy.get_param('~throttle_center', 1532)

SERIAL_PORT = rospy.get_param('~serial_port', '/dev/ttyACM0')
SERIAL_SPEED_BAUDS = rospy.get_param('~serial_speed_bauds', 57600)

# endregion

# region Serial port

ser = serial.Serial(SERIAL_PORT, SERIAL_SPEED_BAUDS)
rospy.loginfo('Connected to serial port %s at %d bauds.', SERIAL_PORT, SERIAL_SPEED_BAUDS)

# endregion


def controller():
    publisher_throttle_microseconds = rospy.Publisher('throttle/microseconds', UInt16, queue_size=1)
    publisher_steering_microseconds = rospy.Publisher('steering/microseconds', UInt16, queue_size=1)
    publisher_throttle_normalized = rospy.Publisher('throttle/normalized', Float32, queue_size=1)
    publisher_steering_normalized = rospy.Publisher('steering/normalized', Float32, queue_size=1)

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

        rospy.logdebug('Steering = %d microseconds, Throttle = %d microseconds', steering, throttle)

        publisher_throttle_microseconds.publish(UInt16(throttle))
        publisher_steering_microseconds.publish(UInt16(steering))

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

        publisher_throttle_normalized.publish(Float32(throttle_normalized))
        publisher_steering_normalized.publish(Float32(steering_normalized))


if __name__ == '__main__':

    try:
        controller()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down...')
