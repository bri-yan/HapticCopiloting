import random
import sys
import time
from collections import deque
import hashlib

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType, CommandType, TwidID

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'

async def serial_test(twid:TwidSerialInterfaceProtocol):
    tid = TwidID.TWID1_ID
    assert await twid.update_telem_sample_rate(tid, 5)
    assert await twid.update_telem_sample_rate(tid, 10)
    assert await twid.update_telem_sample_rate(tid, 20)
    assert await twid.update_telem_sample_rate(tid, 100)
    assert await twid.update_telem_sample_rate(tid, 20)

    for val in [e for e in ControlType]:
        print(f'val:{val}')
        assert await twid.update_control_type(tid, val)
        assert await twid.wait_for_param('control_type', val, tid)
        print(f'actual val:{val}')

    # tid = TwidID.TWID2_ID
    # for val in [e for e in ControlType]:
    #     print(f'val:{val}')
    #     assert await twid.update_control_type(tid, val)
    #     assert await twid.wait_for_param('control_type', val, tid)
    #     print(f'actual val:{val}')

    # assert await twid.update_telem_sample_rate(TwidID.TWID2_ID, 20)
    assert await twid.update_telem_sample_rate(TwidID.TWID1_ID, 20)
    out = await twid.collect_telem(None, 1, False)
    print(out)
    
    await twid.end()

run_test(SERIAL_PORT, serial_test)
