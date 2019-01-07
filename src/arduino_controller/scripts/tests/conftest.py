#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import platform


def runs_on_raspberry_pi():
    return 'arm' in platform.machine()
