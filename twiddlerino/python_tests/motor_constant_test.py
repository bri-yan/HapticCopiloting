import random
import sys
import time
import datetime
from collections import deque

import numpy as np
import scipy

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

#MOTOR EMPIRCAL PARAMS

#voltage supply
PWM_AMPLITUDE = 12

async def test(twid:TwidSerialInterfaceProtocol):
    await twid.update_telem_sample_rate(20)
    await twid.update_control_type(ControlType.NO_CTRL)
    await twid.update_dutycycle(0)

    dcspan = np.linspace(0,1024,100)
    data = {'dc':[], 'volts_approx':[], 'current':[], 'vel_filtered':[],'i_rpm':[],'v_rpm':[],'Ke':[],'R':[]}

    for i in range(dcspan.size):
        await twid.update_dutycycle(int(dcspan[i]))
        await twid.wait_for_param('pwm_duty_cycle', int(dcspan[i]))
        
        #wait 1 sec until steady state
        await asyncio.sleep(1.0)

        data['dc'].append(twid.last_frame.pwm_duty_cycle)
        data['volts_approx'].append(twid.last_frame.pwm_duty_cycle/1024 * PWM_AMPLITUDE)
        data['current'].append(twid.last_frame.current)
        data['vel_filtered'].append(twid.last_frame.filtered_velocity)
        print(f'progress:{i/dcspan.size*100:.2f} %\tdc:{data['dc'][-1]}\tamps:{data['current'][-1]}\trpm:{data['vel_filtered'][-1]}')
    await twid.end_test()

    volts = np.array(data['volts_approx'])
    current = np.array(data['current'])
    rpm = np.array(data['vel_filtered'])
    rads = rpm*0.1047198
    v_curve = volts
    i_curve = current
    for i in range(volts.size):
        if rads[i] != 0:
            v_curve[i] = v_curve[i]/rads[i]
            i_curve[i] = i_curve[i]/rads[i]
        else:
            v_curve[i] = 0
            i_curve[i] = 0

    slope, intercept, r, p, se = scipy.stats.linregress(i_curve[v_curve>=0], v_curve[v_curve>=0]) #linear regression to find slope and intercept
    K = intercept
    R = slope
    data['Ke'].append(K)#slope of curve is motor constant
    data['R'].append(R)#resistance in intercept
    
    torque = current*K
    Bp, Bcp, r, p, se = scipy.stats.linregress(rads[rads >= 0], torque[rads >= 0])
    # Bn, Bcn, r, p, se = scipy.stats.linregress(rads[rads <= 0], torque[rads <= 0])
    data['B'].append(Bp)
    # data['B'].append(Bn)
    data['Bc'].append(Bcp)
    # data['Bc'].append(Bcn)

    print(f'Calibration Complete:\n\
          Motor Constant:\t{data['Ke']} V.s/rad\n\
          Winding Resistance:\t{data['R']} Ohms\n\
          Friction (Linear):\t{data['B']} Nms\n\
          Friction (Coulumb/Static)\t{data['Bc']} Nm\n')

    with open(f'motor_speed_cst_tbl.txt', "w") as f:
        for key,val in data:
            f.write(f'{key}\n')
            for element in val:
                f.write(f'{element},\n')

run_test(SERIAL_PORT, SERIAL_BAUD_RATE, test)

