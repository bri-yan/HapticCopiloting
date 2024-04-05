import numpy as np
import asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType
import os
import time

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 100000

DATA_DIR_PATH = os.path.join(os.getcwd(),'data')

async def ref_track_test(twid:TwidSerialInterfaceProtocol):
    await twid.esp32_reboot()
    await twid.update_telem_sample_rate(1)
    await twid.update_control_type(ControlType.NO_CTRL)
    await twid.wait_for_param('control_type', ControlType.NO_CTRL)
    
    t = np.linspace(0, 25, int(25/0.001))
    omega = np.pi * 2 #rad/s
    setpoint_signal = np.sin(omega*t)
    
    for i in range(len(t)):
        start = time.time_ns()
        await twid.update_setpoint(setpoint_signal[i])
        # await twid.wait_for_param('position_setpoint',setpoint_signal[i])
        print((time.time_ns() - start)*1E-9)
    
    await asyncio.sleep(5)

    await twid.end()
    
run_test(SERIAL_PORT, ref_track_test)