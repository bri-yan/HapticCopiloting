import random
import sys
import time
import datetime
from collections import deque

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM3'
SERIAL_BAUD_RATE = 1000000

async def run_test():
    global twid, loop
    transport, twid = await serial_asyncio.create_serial_connection(loop, TwidSerialInterfaceProtocol, SERIAL_PORT, baudrate=SERIAL_BAUD_RATE)
    await asyncio.sleep(0.5) #wait for connection to init
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
    twid.update_pwm(0)

    with open(f'{datetime.time.__repr__()}_motor_current_calibration_table.txt', "w") as f:
        f.write('duty cycle (0-1024)\n')
        for val in dc:
            f.write(f'{val},\n')
        f.write('current (A)\n')
        for val in current:
            f.write(f'{val},\n')

twid: TwidSerialInterfaceProtocol
loop = asyncio.get_event_loop()
loop.run_until_complete(run_test())
loop.close()

