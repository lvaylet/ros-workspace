#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Use the Adafruit PCA9685 PWM controller library to steer the wheels of the RC car from left to right.
"""

import sys

import Adafruit_PCA9685
import roslib
import rospy
from std_msgs.msg import Float32

import helpers

roslib.load_manifest('pca9685')

# region Constants

PWM_FREQUENCY_HZ = rospy.get_param('~pwm_frequency_hz', 50)  # 50 Hz is suitable for most servos with 20 ms frames

STEERING_MIN_USECS = rospy.get_param('~steering_min_usecs', 1000)
STEERING_CENTER_USECS = rospy.get_param('~steering_center_usecs', 1496)
STEERING_MAX_USECS = rospy.get_param('~steering_max_usecs', 1984)

THROTTLE_MIN_USECS = rospy.get_param('~throttle_min_usecs', 1040)
THROTTLE_CENTER_USECS = rospy.get_param('~throttle_center_usecs', 1532)
THROTTLE_MAX_USECS = rospy.get_param('~throttle_max_usecs', 1996)

STEERING_LIMIT = rospy.get_param('~steering_limit', 1.0)
THROTTLE_LIMIT = rospy.get_param('~throttle_limit', 0.2)  # limit throttle to 20% by default

STEERING_CHANNEL_ON_PCA9685 = rospy.get_param('~steering_channel', 0)
THROTTLE_CHANNEL_ON_PCA9685 = rospy.get_param('~throttle_channel', 1)

# endregion


# region Classes

class ThrottleDriver:

    def __init__(self):
        self.subscriber = rospy.Subscriber('throttle/normalized', Float32, self.callback)

    def callback(self, setpoint):
        # TODO Publish intermediate values to topics for easier troubleshooting
        throttle_normalized = float(setpoint.data)  # a Float32 between 0.0 and 1.0

        if throttle_normalized > THROTTLE_LIMIT:
            throttle_normalized_limited = THROTTLE_LIMIT
        elif throttle_normalized < -THROTTLE_LIMIT:
            throttle_normalized_limited = -THROTTLE_LIMIT
        else:
            throttle_normalized_limited = throttle_normalized

        throttle_microseconds = helpers.normalized_to_microseconds(throttle_normalized_limited,
                                                                   low=THROTTLE_MIN_USECS,
                                                                   center=THROTTLE_CENTER_USECS,
                                                                   high=THROTTLE_MAX_USECS)

        throttle_ticks = helpers.pulse_width_microseconds_to_ticks(throttle_microseconds)

        rospy.loginfo('Throttle normalized: %f, normalized limited: %f, microseconds: %d, ticks: %d',
                      throttle_normalized,
                      throttle_normalized_limited,
                      throttle_microseconds,
                      throttle_ticks)

        pca9685.set_pwm(channel=THROTTLE_CHANNEL_ON_PCA9685,
                        on=0,
                        off=throttle_ticks)


class SteeringDriver:

    def __init__(self):
        self.subscriber = rospy.Subscriber('steering/normalized', Float32, self.callback)

    def callback(self, setpoint):
        # TODO Publish intermediate values to topics for easier troubleshooting
        steering_normalized = float(setpoint.data)  # a Float32 between 0.0 and 1.0

        if steering_normalized > STEERING_LIMIT:
            steering_normalized_limited = STEERING_LIMIT
        elif steering_normalized < -STEERING_LIMIT:
            steering_normalized_limited = -STEERING_LIMIT
        else:
            steering_normalized_limited = steering_normalized

        steering_microseconds = helpers.normalized_to_microseconds(steering_normalized_limited,
                                                                   low=STEERING_MIN_USECS,
                                                                   center=STEERING_CENTER_USECS,
                                                                   high=STEERING_MAX_USECS)

        steering_ticks = helpers.pulse_width_microseconds_to_ticks(steering_microseconds)

        rospy.loginfo('Steering normalized: %f, normalized limited: %f, microseconds: %d, ticks: %d',
                      steering_normalized,
                      steering_normalized_limited,
                      steering_microseconds,
                      steering_ticks)

        pca9685.set_pwm(channel=STEERING_CHANNEL_ON_PCA9685,
                        on=0,
                        off=steering_ticks)

# endregion


# region PCA9685

rospy.loginfo('Connecting to PCA9685...')
pca9685 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

rospy.loginfo('Setting PCA9685 PWM frequency to [%d] Hz...', PWM_FREQUENCY_HZ)
pca9685.set_pwm_freq(PWM_FREQUENCY_HZ)

# endregion


def main(args):
    td = ThrottleDriver()
    sd = SteeringDriver()
    rospy.init_node('pca9685', anonymous=True)
    try:
        rospy.loginfo(
            'IMPORTANT Please secure the RC car before proceeding, for example by putting it on top of a shoebox '
            'or a small but thick book. The wheels should not touch anything.')

        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo('Shutting down...')


if __name__ == '__main__':
    main(sys.argv)
