import random
import sys
import time
import datetime
from collections import deque

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

async def test(twid:TwidSerialInterfaceProtocol):
    twid.turn_off_control()
    twid.update_pwm(0)
    await asyncio.sleep(0.5)

    dcspan = np.linspace(0,1024,100)
    dc = []
    current = []

    for i in range(dcspan.size):
        twid.update_pwm(dcspan[i])
        await asyncio.sleep(1)
        dc.append(twid.last_frame.pwm_duty_cycle)
        current.append(twid.last_frame.current)
        print(dc[-1],current[-1])
    twid.end_test()

    with open(f'{datetime.time.__repr__()}_motor_current_calibration_table.txt', "w") as f:
        f.write('duty cycle (0-1024)\n')
        for val in dc:
            f.write(f'{val},\n')
        f.write('current (A)\n')
        for val in current:
            f.write(f'{val},\n')

run_test(test, SERIAL_PORT, SERIAL_BAUD_RATE)

