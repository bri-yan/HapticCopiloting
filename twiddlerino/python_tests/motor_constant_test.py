import random
import sys
import time
import datetime
from collections import deque

import numpy as np
import scipy

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM3'
SERIAL_BAUD_RATE = 1000000
#MOTOR EMPIRCAL PARAMS
WINDING_RESISTANCE = 1 #OHM
#voltage supply
VSS = 12

async def run_test():
    global twid, loop
    transport, twid = await serial_asyncio.create_serial_connection(loop, TwidSerialInterfaceProtocol, SERIAL_PORT, baudrate=SERIAL_BAUD_RATE)
    await asyncio.sleep(0.5) #wait for connection to init
    twid.turn_off_control()
    twid.update_pwm(0)
    await asyncio.sleep(0.5)

    dcspan = np.linspace(0,1024,100)
    data = {'dc':[], 'volts_approx':[], 'current':[], 'vel_filtered':[],'i_rpm':[],'v_rpm':[],'Ke':[],'R':[]}

    for i in range(dcspan.size):
        twid.update_pwm(dcspan[i])
        await asyncio.sleep(1)
        data['dc'].append(twid.last_frame.pwm_duty_cycle)
        data['volts_approx'].append(twid.last_frame.pwm_duty_cycle/1024 * VSS)
        data['current'].append(twid.last_frame.current)
        data['vel_filtered'].append(twid.last_frame.filtered_velocity)
        print(f'progress:{i/dcspan.size*100} %\tdc:{data['dc'][-1]}\tamps:{data['current'][-1]}\trpm:{data['vel_filtered'][-1]}')
    twid.write(b'stop\n')

    v_rpm = np.divide(np.array(data['volts_approx']),np.array(data['vel_filtered']))
    i_rpm = np.divide(np.array(data['current']),np.array(data['vel_filtered']))
    slope, intercept, r, p, se = scipy.stats.linregress(i_rpm, v_rpm) #linear regression to find slope and intercept
    data['Ke'].append(slope)#slope of curve is motor constant
    data['R'].append(intercept)#resistance in intercept
    print(f'Calibration Complete:\nMotor Constant:{data['Ke']} V.s/rad\tWinding Resistance:{data['R']} Ohms')

    with open(f'{datetime.time.__repr__()}_motor_speed_cst_tbl.txt', "w") as f:
        for key,val in data:
            f.write(f'{key}\n')
            for element in val:
                f.write(f'{element},\n')

twid: TwidSerialInterfaceProtocol
loop = asyncio.get_event_loop()
loop.run_until_complete(run_test())
loop.close()

