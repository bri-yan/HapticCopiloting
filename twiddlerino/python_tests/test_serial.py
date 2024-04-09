import random
import sys
import time
from collections import deque
import hashlib

import numpy as np

import serial

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType, CommandType, TwidID

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = "/dev/tty.usbserial-1440"
baudrate = 9600

try:
    # Open the serial port
    ser = serial.Serial(SERIAL_PORT, baudrate)
    
    # Check if the port is open
    if ser.is_open:
        print(f"Serial port {SERIAL_PORT} is open.")
    else:
        print(f"Failed to open serial port {SERIAL_PORT}.")

except Exception as e:
    print(f"Error: {e}")

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
