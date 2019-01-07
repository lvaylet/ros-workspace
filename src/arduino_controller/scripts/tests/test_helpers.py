#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from helpers import map_to_range, pulse_width_microseconds_to_ticks, microseconds_to_normalized, \
    normalized_to_microseconds, serial_data_to_dict


class TestMapToRange:

    def test_0_microseconds_should_map_to_0_ticks_at_50_hz(self):
        # Map from [0; 20 ms] to [0; 4095] (20 ms because of 50 Hz)
        assert map_to_range(0, 0, 20000, 0, 4095) == 0

    def test_2_microseconds_should_map_to_0_ticks_at_50_hz(self):
        assert map_to_range(2, 0, 20000, 0, 4095) == 0

    def test_7_microseconds_should_map_to_1_ticks_at_50_hz(self):
        assert map_to_range(7, 0, 20000, 0, 4095) == 1

    def test_20000_microseconds_should_map_to_4095_ticks_at_50_hz(self):
        assert map_to_range(20000, 0, 20000, 0, 4095) == 4095


class TestMicrosecondsToTicks:

    def test_0_microseconds_should_be_0_ticks_at_50_hz(self):
        assert pulse_width_microseconds_to_ticks(0, 50) == 0

    def test_0_microseconds_should_be_0_ticks_at_1000_hz(self):
        assert pulse_width_microseconds_to_ticks(0, 1000) == 0

    def test_20000_microseconds_should_be_4095_ticks_at_50_hz(self):
        assert pulse_width_microseconds_to_ticks(20000, 50) == 4095

    def test_10_microseconds_should_be_40_ticks_at_1000_hz(self):
        assert pulse_width_microseconds_to_ticks(10, 1000) == 40

    def test_1000_microseconds_should_be_204_ticks_at_1000_hz(self):
        assert pulse_width_microseconds_to_ticks(1000, 50) == 204

    def test_1500_microseconds_should_be_307_ticks_at_1000_hz(self):
        assert pulse_width_microseconds_to_ticks(1500, 50) == 307

    def test_2000_microseconds_should_be_409_ticks_at_50_hz(self):
        assert pulse_width_microseconds_to_ticks(2000, 50) == 409


class TestNormalize:

    def test_1496_microseconds_steering_should_normalize_to_0(self):
        result = microseconds_to_normalized(1496, 1000, 1496, 1984)
        assert type(result) == float
        assert result == 0.0

    def test_1000_microseconds_steering_should_normalize_to_minus_1(self):
        result = microseconds_to_normalized(1000, 1000, 1496, 1984)
        assert type(result) == float
        assert result == -1.0

    def test_1984_microseconds_steering_should_normalize_to_plus_1(self):
        result = microseconds_to_normalized(1984, 1000, 1496, 1984)
        assert type(result) == float
        assert result == +1.0

    def test_1740_microseconds_steering_should_normalize_to_one_half(self):
        result = microseconds_to_normalized(1740, 1000, 1496, 1984)
        assert type(result) == float
        assert result == +0.5


class TestDenormalize:

    def test_0_steering_should_denormalize_to_1496_microseconds(self):
        assert normalized_to_microseconds(0.0, 1000, 1496, 1984) == 1496

    def test_plus_1_steering_should_denormalize_to_1984_microseconds(self):
        assert normalized_to_microseconds(+1.0, 1000, 1496, 1984) == 1984

    def test_minus_1_steering_should_denormalize_to_1000_microseconds(self):
        assert normalized_to_microseconds(-1.0, 1000, 1496, 1984) == 1000

    def test_out_of_range_setpoints_should_denormalize_to_1496_microseconds(self):
        assert normalized_to_microseconds(-1.2, 1000, 1496, 1984) == 1496
        assert normalized_to_microseconds(-5.8, 1000, 1496, 1984) == 1496
        assert normalized_to_microseconds(+1.2, 1000, 1496, 1984) == 1496
        assert normalized_to_microseconds(+5.8, 1000, 1496, 1984) == 1496


class TestNormalizeAndDenormalize:

    def test_1496_should_normalize_then_denormalize_to_1496(self):
        assert normalized_to_microseconds(microseconds_to_normalized(1496, 1000, 1496, 1984), 1000, 1496, 1984) == 1496

    def test_1984_should_normalize_then_denormalize_to_1984(self):
        assert normalized_to_microseconds(microseconds_to_normalized(1984, 1000, 1496, 1984), 1000, 1496, 1984) == 1984

    def test_1000_should_normalize_then_denormalize_to_1000(self):
        assert normalized_to_microseconds(microseconds_to_normalized(1000, 1000, 1496, 1984), 1000, 1496, 1984) == 1000


class TestSerialDataToDict:

    def test_valid_byte_array_should_return_valid_dict(self):
        assert serial_data_to_dict(b'CH1:1400\tCH2:1600\n') == {'CH1': 1400, 'CH2': 1600}
