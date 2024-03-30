import random
import sys
import time
import datetime
from collections import deque

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType

#control parameter fitting
from tbcontrol.responses import fopdt, sopdt
import scipy.optimize

import matplotlib.pyplot as plt

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

#MOTOR EMPIRCAL PARAMS

#voltage supply
PWM_AMPLITUDE = 12

async def test(twid:TwidSerialInterfaceProtocol):
    await twid.update_telem_sample_rate(50)
    
    await twid.update_control_type(ControlType.NO_CTRL)
    await twid.wait_for_param('control_type', ControlType.NO_CTRL)

    await twid.update_dutycycle(0)
    await twid.wait_for_param('pwm_duty_cycle',0)

    #apply step response (step duty cycle from 0 to 512/1024 ~ 50% voltage)
    await twid.update_dutycycle(512)
    #collect all data for next 5 seconds
    data = await twid.collect_telem(5)
    
    #extract time and data arrays
    ts = []
    ym = []
    x = []
    start_ms = data[0].timestamp_ms
    for t in data:
        x.append(t.pwm_duty_cycle)
        ts.append(t.timestamp_ms - start_ms)
        ym.append(t.filtered_current)
    x = np.array(x)
    ts = np.array(ts)
    ym = np.array(ym)
    
    #apply curve fitting to get second order params
    [K, tau, zeta, theta, y0], _ = scipy.optimize.curve_fit(sopdt, ts[x>=500], ym[x>=500], [2, 2, 1.5, 1, 0])
    print(f'Step response estimated second order parameters:\n\
          K:{K}\ttau;{tau}\tzeta:{zeta}\ttheta{theta}\ty0:{y0}')

    plt.figure(figsize=(10, 5))
    plt.scatter(ts, ym, label='Data')
    plt.plot(ts, sopdt(ts, K, tau, zeta, theta, y0), color='red', label='SOPDT fit')
    plt.legend(loc='best')
    plt.savefig('step_resp_fit.png')
    plt.close()

    await twid.end_test()

run_test(test, SERIAL_PORT, SERIAL_BAUD_RATE)

