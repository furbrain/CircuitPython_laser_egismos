# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Phil Underwood for Underwood Underground
#
# SPDX-License-Identifier: Unlicense
import time

import board
import busio
import digitalio

from laser_egismos import Laser

periph_power = digitalio.DigitalInOut(board.D5)
periph_power.switch_to_output(False)
laser_power = digitalio.DigitalInOut(board.D0)
laser_power.switch_to_output(False)

time.sleep(0.3)
print("power on")
periph_power.switch_to_output(True)
print("laser power enable")
laser_power.switch_to_output(True)
time.sleep(0.1)
uart = busio.UART(board.D2, board.D1, baudrate=9600)
print("start talking")
laser = Laser(uart)
laser.set_buzzer(False)
laser.set_laser(True)
time.sleep(1)
laser.set_laser(False)
time.sleep(0.1)
print(f"Distance is {laser.distance}cm")
