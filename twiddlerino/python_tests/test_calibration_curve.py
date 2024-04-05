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
    await twid.update_telem_sample_rate(50)
    await twid.update_control_type(ControlType.NO_CTRL)
    await twid.update_dutycycle(0)

    dcspan = np.linspace(0,1024,100)
    dc = []
    current = []

    for i in range(dcspan.size):
        await twid.update_dutycycle(int(dcspan[i]))
        await twid.wait_for_param('pwm_duty_cycle', int(dcspan[i]))
        
        #wait 1 sec until steady state
        await asyncio.sleep(1.0)
        
        frame = twid.last_frame
        dc.append(frame.pwm_duty_cycle)
        current.append(frame.current)
        print(f'dutycycle: {dc[-1]:.2f}\tcurrent: {current[-1]:.3f}')
    await twid.end()

    with open(f'motor_current_calibration_table.txt', "w") as f:
        f.write('duty cycle (0-1024)\n')
        for val in dc:
            f.write(f'{val},\n')
        f.write('current (A)\n')
        for val in current:
            f.write(f'{val},\n')

run_test(SERIAL_PORT, test)

