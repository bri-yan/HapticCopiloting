import random
import sys
import time
from collections import deque

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

async def test(twid:TwidSerialInterfaceProtocol):
    twid.update_telem_sample_rate(20)
    await asyncio.sleep(0.5)
    start = time.time()
    while time.time() < start + 1000:
        await asyncio.sleep(0.001)
    twid.end_test()

run_test(test, SERIAL_PORT, SERIAL_BAUD_RATE)
