#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Use the Adafruit PCA9685 PWM controller library to steer the wheels of the RC car from left to right.
"""

import os
import sys

import Adafruit_PCA9685
import roslib
import rospy
from std_msgs.msg import Float32

import helpers

roslib.load_manifest('pca9685')

# region Constants

PWM_FREQUENCY_HZ = int(os.environ.get('PWN_FREQUENCY_HZ', '50'))  # 50 Hz is suitable for most servos with 20 ms frames

STEERING_MIN = int(os.environ.get('STEERING_MIN', '1000'))
STEERING_MAX = int(os.environ.get('STEERING_MAX', '1984'))
STEERING_CENTER = int(os.environ.get('STEERING_CENTER', '1496'))

THROTTLE_MIN = int(os.environ.get('THROTTLE_MIN', '1040'))
THROTTLE_MAX = int(os.environ.get('THROTTLE_MAX', '1996'))
THROTTLE_CENTER = int(os.environ.get('THROTTLE_CENTER', '1532'))

STEERING_LIMIT = float(os.environ.get('STEERING_LIMIT', '1.0'))
THROTTLE_LIMIT = float(os.environ.get('THROTTLE_LIMIT', '0.1'))  # limit throttle to 10% by default

STEERING_CHANNEL_ON_PCA9685 = int(os.environ.get('STEERING_CHANNEL_ON_PCA9685', '0'))
THROTTLE_CHANNEL_ON_PCA9685 = int(os.environ.get('THROTTLE_CHANNEL_ON_PCA9685', '1'))

# endregion

# region Classes


class ThrottleDriver:

    def __init__(self):
        self.subscriber = rospy.Subscriber('throttle/normalized', Float32, self.callback)

    def callback(self, data):
        # TODO Publish intermediate values to topics for easier troubleshooting
        throttle_normalized = data  # a Float32 between 0.0 and 1.0

        if throttle_normalized > THROTTLE_LIMIT:
            throttle_normalized_limited = THROTTLE_LIMIT
        elif throttle_normalized < -THROTTLE_LIMIT:
            throttle_normalized_limited = -THROTTLE_LIMIT
        else:
            throttle_normalized_limited = throttle_normalized

        throttle_limited = helpers.normalized_to_microseconds(throttle_normalized_limited,
                                                              low=THROTTLE_MIN,
                                                              center=THROTTLE_CENTER,
                                                              high=THROTTLE_MAX)

        pca9685.set_pwm(channel=THROTTLE_CHANNEL_ON_PCA9685,
                        on=0,
                        off=helpers.pulse_width_microseconds_to_ticks(throttle_limited))


class SteeringDriver:

    def __init__(self):
        self.subscriber = rospy.Subscriber('steering/normalized', Float32, self.callback)

    def callback(self, data):
        # TODO Publish intermediate values to topics for easier troubleshooting
        steering_normalized = data  # a Float32 between 0.0 and 1.0

        if steering_normalized > STEERING_LIMIT:
            steering_normalized_limited = STEERING_LIMIT
        elif steering_normalized < -STEERING_LIMIT:
            steering_normalized_limited = -STEERING_LIMIT
        else:
            steering_normalized_limited = steering_normalized

        steering_limited = helpers.normalized_to_microseconds(steering_normalized_limited,
                                                              low=STEERING_MIN,
                                                              center=STEERING_CENTER,
                                                              high=STEERING_MAX)

        pca9685.set_pwm(channel=STEERING_CHANNEL_ON_PCA9685,
                        on=0,
                        off=helpers.pulse_width_microseconds_to_ticks(steering_limited))


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
        input('Press Enter to continue...')

        rospy.spin()
    
    except KeyboardInterrupt:
        print('Shutting down...')


if __name__ == '__main__':
    main(sys.argv)
