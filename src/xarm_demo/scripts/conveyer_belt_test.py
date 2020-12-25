#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Set Controller GPIO Digital/Analog
"""

import os
import sys
import time

from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.18.220')
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

arm.set_cgpio_digital(ionum=4,value=1)
arm.set_cgpio_digital(ionum=5,value=0)

time.sleep(13)

arm.set_cgpio_digital(ionum=4,value=1)
arm.set_cgpio_digital(ionum=5,value=1)
