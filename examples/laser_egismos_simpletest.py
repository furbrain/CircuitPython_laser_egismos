# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Phil Underwood for Underwood Underground
#
# SPDX-License-Identifier: Unlicense
import time

import board
import busio
import digitalio

from laser_egismos import Laser, LaserCommandFailedError

laser_power = digitalio.DigitalInOut(board.D10)
laser_power.switch_to_output(True)


uart = busio.UART(board.D8, board.D9, baudrate=9600)
laser = Laser(uart)
print(laser._send_and_receive(laser.READ_SW_VERSION))
laser.buzzer_off()
laser.laser_on()
time.sleep(3)
laser.laser_off()
time.sleep(0.1)
print(f"Distance is {laser.distance}cm")
