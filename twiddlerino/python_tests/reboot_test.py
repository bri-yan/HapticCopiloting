import random
import sys
import time
from collections import deque
import hashlib

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_test, ControlType, CommandType

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

async def serial_test(twid:TwidSerialInterfaceProtocol):
    assert await twid.update_telem_sample_rate(20)
    
    for i in range(100):
        print(f'reboot cycle {i}/100')
        assert await twid.esp32_reboot()
    
    await twid.end()

run_test(SERIAL_PORT, SERIAL_BAUD_RATE, serial_test)
