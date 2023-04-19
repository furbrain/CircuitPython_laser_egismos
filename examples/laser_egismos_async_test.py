# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Phil Underwood for Underwood Underground
#
# SPDX-License-Identifier: Unlicense

import asyncio
import board
import busio
import digitalio
from laser_egismos import AsyncLaser


periph_power = digitalio.DigitalInOut(board.D5)
periph_power.switch_to_output(True)
laser_power = digitalio.DigitalInOut(board.D0)
laser_power.switch_to_output(True)


async def counter():
    for i in range(15):
        print(f"count: {i}")
        await asyncio.sleep(1)


async def reader():
    await asyncio.sleep(0.5)
    uart = busio.UART(board.D2, board.D1, baudrate=9600)
    laser = AsyncLaser(uart)
    await laser.set_buzzer(False)
    for _ in range(5):
        distance = await laser.measure()
        print(f"distance: {distance}")


async def main():
    await asyncio.gather(counter(), reader())


asyncio.run(main())
