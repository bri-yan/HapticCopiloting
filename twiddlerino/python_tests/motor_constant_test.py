import random
import sys
import time
import datetime
from collections import deque

import numpy as np
import scipy

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

#MOTOR EMPIRCAL PARAMS

#voltage supply
PWM_AMPLITUDE = 12

async def test(twid:TwidSerialInterfaceProtocol):
    twid.turn_off_control()
    twid.update_pwm(0)
    await asyncio.sleep(0.5)

    dcspan = np.linspace(0,1024,100)
    data = {'dc':[], 'volts_approx':[], 'current':[], 'vel_filtered':[],'i_rpm':[],'v_rpm':[],'Ke':[],'R':[]}

    for i in range(dcspan.size):
        twid.update_pwm(dcspan[i])
        await asyncio.sleep(0.1)
        data['dc'].append(twid.last_frame.pwm_duty_cycle)
        data['volts_approx'].append(twid.last_frame.pwm_duty_cycle/1024 * PWM_AMPLITUDE)
        data['current'].append(twid.last_frame.current)
        data['vel_filtered'].append(twid.last_frame.filtered_velocity)
        print(f'progress:{i/dcspan.size*100:.2f} %\tdc:{data['dc'][-1]}\tamps:{data['current'][-1]}\trpm:{data['vel_filtered'][-1]}')
    twid.end_test()

    volts = np.array(data['volts_approx'])
    current = np.array(data['current'])
    rpm = np.array(data['vel_filtered'])
    rads = rpm*0.1047198
    rads = rads[rads > 0] ; volts = volts[rads > 0] ; current = current[rads > 0]
    v_curve = np.divide(volts,rads)
    i_curve = np.divide(current,rads)
    slope, intercept, r, p, se = scipy.stats.linregress(i_curve, v_curve) #linear regression to find slope and intercept
    data['Ke'].append(slope)#slope of curve is motor constant
    data['R'].append(intercept)#resistance in intercept
    print(f'Calibration Complete:\nMotor Constant:{data['Ke']} V.s/rad\tWinding Resistance:{data['R']} Ohms')

    with open(f'motor_speed_cst_tbl.txt', "w") as f:
        for key,val in data:
            f.write(f'{key}\n')
            for element in val:
                f.write(f'{element},\n')

run_test(test, SERIAL_PORT, SERIAL_BAUD_RATE)

