import numpy as np

import asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType, CommandType, TwidID

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = '/dev/cu.Bluetooth-Incoming-Port'

async def test(twid:TwidSerialInterfaceProtocol):
    tid = TwidID.TWID1_ID
    assert await twid.update_telem_sample_rate(tid, 50)
    assert await twid.update_control_type(tid, ControlType.NO_CTRL)
    assert await twid.update_dutycycle(tid, 0)

    dcspan = np.linspace(0,1024,50)
    dc = []
    current = []

    for i in range(dcspan.size):
        assert await twid.update_dutycycle(tid, int(dcspan[i]))
        assert await twid.wait_for_param('pwm_duty_cycle', int(dcspan[i]),twid_id=tid)
        
        #wait 1 sec until steady state
        await asyncio.sleep(1.0)
        
        frame = twid.latest_frame_t1
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

