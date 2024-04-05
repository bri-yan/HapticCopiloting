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
    assert await twid.update_telem_sample_rate(5)
    assert await twid.update_telem_sample_rate(10)
    assert await twid.update_telem_sample_rate(20)
    assert await twid.update_telem_sample_rate(100)
    assert await twid.update_telem_sample_rate(20)

    for val in [e for e in ControlType]:
        print(f'val:{val}')
        assert await twid.update_control_type(val)
        assert await twid.wait_for_param('control_type', val)
        print(f'actual val:{val}')

    assert await twid.send_cmd(bytes(f'aggasdfhhsdf\n',"utf-8"), CommandType.NA_CMD)
    assert await twid.send_cmd(bytes(f'set_mode,no_control,\n',"utf-8"), CommandType.SET_MODE, twid_id=TwidID.TWID2_ID)
    assert await twid.send_cmd(bytes(f'set_dutycycle,0,\n',"utf-8"), CommandType.SET_DUTYCYCLE)
    assert await twid.send_cmd(bytes(f'reset\n',"utf-8"), CommandType.RESET)
    assert await twid.send_cmd(bytes(f'reboot\n',"utf-8"), CommandType.REBOOT)

    assert await twid.update_telem_sample_rate(5)
    out = await twid.collect_telem(1, False)
    print(out)
    
    await twid.end()

run_test(SERIAL_PORT, serial_test)
