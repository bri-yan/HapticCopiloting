"""
References:
* umich dc motor system identification with matlab (blockbox methods) https://ctms.engin.umich.edu/CTMS/index.php?aux=Activities_DCmotorA
* (python control library) https://python-control.readthedocs.io/en/0.10.0/

Usage in this file:
* python tbcontrol library usage https://github.com/alchemyst/Dynamics-and-Control/blob/master/TOC.ipynb
* parameter fitting based on step response for first and second order systems 
https://github.com/alchemyst/Dynamics-and-Control/blob/e49e69347e8e67c82d5543b5578989aca014b28f//1_Dynamics/7_System_identification/Dynamic%20model%20parameter%20estimation.ipynb

"""
import numpy as np

import asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType, TwidID

#control parameter fitting
import scipy.optimize
import scipy.signal

import matplotlib.pyplot as plt
import pandas

import os

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM4'

#MOTOR EMPIRCAL PARAMS

#voltage supply
PWM_AMPLITUDE = 12
DC_STEPS = [128, 256, 512, 1024]

DATA_DIR_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),'data')

#second order model
def so_model(x,K,wn,zeta):
    return scipy.signal.lti([K],[1, 2*wn*zeta, wn**2]).step(T=x)[1]

async def current_step_response_test(twid:TwidSerialInterfaceProtocol):
    tid = TwidID.TWID1_ID
    await twid.update_telem_sample_rate(tid, 5)
    await twid.update_control_type(tid,ControlType.NO_CTRL)
    await twid.wait_for_param('control_type', ControlType.NO_CTRL,tid)

    await twid.update_dutycycle(tid,0)
    await twid.wait_for_param('pwm_duty_cycle',0,tid)

    #apply step input and record step response for different step inputs
    for step_input in DC_STEPS:
        await twid.update_dutycycle(tid,step_input)
        #collect all data for next 5 seconds
        dataseries = await twid.collect_telem(tid,5)
        print(dataseries)
        #stop motor
        await twid.motor_stop()
        #sleep for 1 sec to give motor time to settle down
        await asyncio.sleep(1.0)
        
        #save as a nice csv using pandas
        dataseries.to_csv(os.path.join(DATA_DIR_PATH,f'current_step_response_{step_input}.csv'), index=False)

        #collect info from start of step only
        dataseries['timestamp_ms'] = dataseries['timestamp_ms'][dataseries['pwm_duty_cycle'] >= step_input]
        dataseries['current'] = dataseries['current'][dataseries['pwm_duty_cycle'] >= step_input]
    
        #apply curve fitting to get second order params using scipy
        #here we are curve fitting to a second order transfer function with dead time
        #has an initial guess of [2, 2, 1.5, 1, 0]
        [K, wn, zeta],_ = scipy.optimize.curve_fit(so_model, dataseries['timestamp_ms'], dataseries['current'])
        output_str = f'Step response estimated second order parameters:\n\
            K:{K}\twn:{wn}\tzeta:{zeta}\n'
        print(output_str)
        #save params to a text file
        with open(os.path.join(DATA_DIR_PATH,f'so_tf_params_{step_input}.txt'), "w") as f:
            f.write(output_str)

        #make some plots too :)
        plt.figure(figsize=(10, 5))
        #real data
        plt.scatter(dataseries['timestamp_ms'], dataseries['current'], label='Data')
        #fitted transfer function response data
        plt.plot(dataseries['timestamp_ms'], so_model(dataseries['timestamp_ms'], K, wn, zeta), color='red', label='SOPDT TF fit')
        plt.legend(loc='best')
        #save plot
        plt.savefig(os.path.join(DATA_DIR_PATH,f'step_resp_fit_{step_input}.png'))
        plt.close()

    await twid.end()
    
async def position_step_response_test(twid:TwidSerialInterfaceProtocol):
    pass

run_test(SERIAL_PORT, current_step_response_test)

