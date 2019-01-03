#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Use the Adafruit PCA9685 PWM controller library to steer the wheels of the RC car from left to right.
"""

import logging
import os
import time

import Adafruit_PCA9685

from helpers import pulse_width_microseconds_to_ticks

# region Logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# endregion

# region Constants

PWM_FREQUENCY_HZ = int(os.environ.get('PWN_FREQUENCY_HZ', '50'))  # 50 Hz is suitable for most servos with 20 ms frames
STEERING_RIGHT_PULSE_LENGTH_MICROSECONDS = int(os.environ.get('STEERING_MIN', '1000'))
STEERING_LEFT_PULSE_WIDTH_MICROSECONDS = int(os.environ.get('STEERING_MAX', '1984'))
STEERING_CENTER_PULSE_WIDTH_MICROSECONDS = int(os.environ.get('STEERING_CENTER', '1984'))

STEERING_CHANNEL_ON_PCA9685 = int(os.environ.get('STEERING_CHANNEL_ON_PCA9685', '0'))
THROTTLE_CHANNEL_ON_PCA9685 = int(os.environ.get('THROTTLE_CHANNEL_ON_PCA9685', '1'))

# endregion

logger.info('Connecting to PCA9685...')
pca9685 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

logger.info('Setting PCA9685 PWM frequency to [%d] Hz...', PWM_FREQUENCY_HZ)
pca9685.set_pwm_freq(PWM_FREQUENCY_HZ)

logger.info('IMPORTANT Please secure the RC car before proceeding, for example by putting it on top of a shoebox '
            'or a small but thick book. The wheels should not touch anything.')
input('Press Enter to continue...')

logger.info('Steering wheels. Press CTRL+C to exit.')
while True:
    logger.info('Steering left...')
    pca9685.set_pwm(channel=STEERING_CHANNEL_ON_PCA9685,
                    on=0,
                    off=pulse_width_microseconds_to_ticks(STEERING_LEFT_PULSE_WIDTH_MICROSECONDS))
    time.sleep(1)

    logger.info('Going straight ahead...')
    pca9685.set_pwm(channel=STEERING_CHANNEL_ON_PCA9685,
                    on=0,
                    off=pulse_width_microseconds_to_ticks(STEERING_CENTER_PULSE_WIDTH_MICROSECONDS))
    time.sleep(1)

    logger.info('Steering right...')
    pca9685.set_pwm(channel=STEERING_CHANNEL_ON_PCA9685,
                    on=0,
                    off=pulse_width_microseconds_to_ticks(STEERING_RIGHT_PULSE_LENGTH_MICROSECONDS))
    time.sleep(1)

    logger.info('Going straight ahead...')
    pca9685.set_pwm(channel=STEERING_CHANNEL_ON_PCA9685,
                    on=0,
                    off=pulse_width_microseconds_to_ticks(STEERING_CENTER_PULSE_WIDTH_MICROSECONDS))
    time.sleep(1)

