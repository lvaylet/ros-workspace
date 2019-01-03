#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import re
from typing import Dict

logger = logging.getLogger(__name__)


def map_to_range(x: int, from_low: int, from_high: int, to_low: int, to_high: int) -> int:
    """
    Re-map a number from one range to another.
    A value of fromLow would get mapped to toLow, a value of fromHigh to toHigh, values in-between to values in-between.
    Do not constrain values to within the range, because out-of-range values are sometimes intended and useful.
    Inspired by https://www.arduino.cc/reference/en/language/functions/math/map/
    :param x: The number to map
    :param from_low: The lower bound of the value’s current range
    :param from_high: The upper bound of the value’s current range
    :param to_low: The lower bound of the value’s target range
    :param to_high: The upper bound of the value’s target range
    :return: The re-mapped value
    :type x: int
    :type from_low: int
    :type from_high: int
    :type to_low: int
    :type to_high: int
    :rtype: int
    """
    return int((x - from_low) * (to_high - to_low) / (from_high - from_low) + to_low)


# FIXME Review page 17 of https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf for a good example of how to compute ticks
def pulse_width_microseconds_to_ticks(desired_pulse_width_microseconds: int, pwm_freq_hz: int = 50) -> int:
    """
    Convert a PWM pulse width from microseconds to 12-bit ticks (0..4095).
    If setting the pulse width in microseconds is easier, you can do that by first figuring out how long each cycle is.
    That would be 1/pwm_freq_hz. For 1000 Hz, that would be 1 millisecond. For 50 Hz, that would be 20 milliseconds.
    Then divide by 4096 to get the time per tick. That would be 1 millisecond / 4096 = ~0.24 microseconds at 1000 Hz, or
    20 milliseconds / 4096 = ~4.88 microseconds at 50 Hz. If you want a pulse that is 10 microseconds long, divide the
    time by time-per-tick (10 us / 0.24 us = 42). Then turn on at tick 0 and turn off at tick 42.
    Reference: https://cdn-learn.adafruit.com/downloads/pdf/adafruit-16-channel-servo-driver-with-raspberry-pi.pdf
    :param desired_pulse_width_microseconds: The pulse width to convert, in microseconds
    :param pwm_freq_hz: The PWM frequency
    :return: The pulse width in ticks with 12-bit resolution (0..4095)
    :type desired_pulse_width_microseconds: int
    :type pwm_freq_hz: int
    :rtype: int
    """
    microseconds_per_second = 1_000_000
    microseconds_per_period = microseconds_per_second // pwm_freq_hz
    # Now map the desired pulse width (in microseconds) to ticks (0..4095 due to the 12-bit resolution)
    pulse_width_ticks = map_to_range(desired_pulse_width_microseconds, 0, microseconds_per_period, 0, 4095)
    return pulse_width_ticks


def microseconds_to_normalized(reading: int, min_reading: int, center_reading: int, max_reading: int) -> float:
    """
    Converts a pulse width (in microseconds) to a normalized setpoint (between -1.0 and 1.0).
    Positive and negative values are computed separately, as they can span different intervals initially.
    :param reading: The pulse width to convert.
    :param min_reading: The minimum reading, mapped to -1.0.
    :param center_reading: The center reading, at rest, mapped to 0.0.
    :param max_reading: The maximum reading, mapped to 1.0.
    :return: The normalized value.
    :type reading: int
    :type min_reading: int
    :type center_reading: int
    :type max_reading: int
    :rtype: float
    """
    centered_reading = reading - center_reading
    if centered_reading > 0:
        normalized_reading = centered_reading / (max_reading - center_reading)
    else:
        normalized_reading = centered_reading / (center_reading - min_reading)

    return normalized_reading


def normalized_to_microseconds(normalized_setpoint: float, low: int, center: int, high: int) -> int:
    """
    Converts a normalized setpoint (between -1.0 and 1.0) to a pulse width (in microseconds).
    Positive and negative values are computed separately, as they can span different intervals in terms of pulse widths.
    :param normalized_setpoint: The normalized setpoint to convert, between -1.0 and 1.0.
    :param low: The minimum pulse width, in microseconds.
    :param center: The center pulse width, in microseconds.
    :param high: The maximum pulse width, in microseconds.
    :return: The pulse width, in microseconds.
    :type normalized_setpoint: float
    :type low: int
    :type center: int
    :type high: int
    :rtype: int
    """
    if abs(normalized_setpoint) > 1.0:
        return center

    if normalized_setpoint > 0.0:
        return int(normalized_setpoint * (high - center)) + center
    else:
        return int(normalized_setpoint * (center - low)) + center


def serial_data_to_dict(byte_array: bytes) -> Dict[str, int]:
    """
    Convert a byte array to a dictionary like {'channel name': value}
    :param byte_array: The byte array read from the serial port.
    :return: The dictionary.
    :type byte_array: bytes
    :rtype: dict[str, int]
    """
    # Compile a regex that can parse a byte array buffer with an arbitrary number
    # of records, each consisting of a channel name, a colon and a numeric value.
    pattern = re.compile(b'(CH\d+):(\d+)')
    # Parse byte array with regex, as a list of tuples (channel name, value)
    packed = pattern.findall(byte_array)
    # Unpack as a list of tuples
    unpacked = [(x[0].decode(), int(x[1].decode())) for x in packed]
    # Build a dictionary out of the unpacked list of tuples (key = channel name, value = channel value)
    return dict(unpacked)

