import random
import sys
import time
import datetime
from collections import deque

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType, CommandType

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

async def test(twid:TwidSerialInterfaceProtocol):
    await twid.update_telem_sample_rate(5)
    await twid.update_control_type(ControlType.NO_CTRL)
    await twid.update_dutycycle(0)

    dcspan = np.linspace(0,1024,100)
    dc = []
    current = []

    for i in range(dcspan.size):
        print(dcspan[i])
        await twid.update_dutycycle(dcspan[i])
        await asyncio.sleep(0.5)

        dc.append(twid.last_frame.pwm_duty_cycle)
        current.append(twid.last_frame.current)
        print(dc[-1],current[-1])
    await twid.end_test()

    with open(f'motor_current_calibration_table.txt', "w") as f:
        f.write('duty cycle (0-1024)\n')
        for val in dc:
            f.write(f'{val},\n')
        f.write('current (A)\n')
        for val in current:
            f.write(f'{val},\n')

run_test(test, SERIAL_PORT, SERIAL_BAUD_RATE)

