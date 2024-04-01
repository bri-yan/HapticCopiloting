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
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType

#control parameter fitting
from tbcontrol.responses import fopdt, sopdt
import scipy.optimize

import matplotlib.pyplot as plt
import pandas

import os

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

#MOTOR EMPIRCAL PARAMS

#voltage supply
PWM_AMPLITUDE = 12
DC_STEPS = [128, 256, 512, 1024]

DATA_DIR_PATH = os.path.join(os.getcwd(),'data')

async def current_step_response_test(twid:TwidSerialInterfaceProtocol):
    await twid.update_telem_sample_rate(20)
    await twid.update_control_type(ControlType.NO_CTRL)
    await twid.wait_for_param('control_type', ControlType.NO_CTRL)

    await twid.update_dutycycle(0)
    await twid.wait_for_param('pwm_duty_cycle',0)

    #apply step input and record step response for different step inputs
    for step_input in DC_STEPS:
        await twid.update_dutycycle(step_input)
        #collect all data for next 5 seconds
        data = await twid.collect_telem(5)
        #stop motor
        await twid.motor_stop()
        #sleep for 1 sec to give motor time to settle down
        await asyncio.sleep(1.0)
    
        #extract time and data arrays
        dataseries = {'ts':[], 'current':[], 'dc':[]}
        start_ms = data[0].timestamp_ms
        for t in data:
            dataseries['dc'].append(t.pwm_duty_cycle)
            dataseries['ts'].append(t.timestamp_ms - start_ms)
            dataseries['current'].append(t.filtered_current)
        for key in dataseries.keys():
            dataseries[key] = np.array(dataseries[key])
        
        #save as a nice csv using pandas
        pandas.DataFrame(dataseries).to_csv(os.path.join(DATA_DIR_PATH,f'current_step_response_{step_input}.csv'), index=False)

        #collect info from start of step only
        dataseries['ts'] = dataseries['ts'][dataseries['dc'] >= step_input]
        dataseries['current'] = dataseries['current'][dataseries['dc'] >= step_input]
    
        #apply curve fitting to get second order params using scipy
        #here we are curve fitting to a second order transfer function with dead time
        #has an initial guess of [2, 2, 1.5, 1, 0]
        [K, tau, zeta, theta, y0], _ = scipy.optimize.curve_fit(sopdt, dataseries['ts'], dataseries['current'], [2, 2, 1.5, 1, 0])
        output_str = f'Step response estimated second order parameters:\n\
            K:{K}\ttau;{tau}\tzeta:{zeta}\ttheta{theta}\ty0:{y0}\n'
        print(output_str)
        #save params to a text file
        with open(os.path.join(DATA_DIR_PATH,f'so_tf_params_{step_input}.txt'), "w") as f:
            f.write(output_str)

        #make some plots too :)
        plt.figure(figsize=(10, 5))
        #real data
        plt.scatter(dataseries['ts'], dataseries['current'], label='Data')
        #fitted transfer function response data
        plt.plot(dataseries['ts'], sopdt(dataseries['ts'], K, tau, zeta, theta, y0), color='red', label='SOPDT TF fit')
        plt.legend(loc='best')
        #save plot
        plt.savefig(os.path.join(DATA_DIR_PATH,f'step_resp_fit_{step_input}.png'))
        plt.close()

    await twid.end_test()
    
async def position_step_response_test(twid:TwidSerialInterfaceProtocol):
    pass

run_test(current_step_response_test, SERIAL_PORT, SERIAL_BAUD_RATE)

