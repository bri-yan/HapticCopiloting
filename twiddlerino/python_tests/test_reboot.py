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

async def reboot_test(twid:TwidSerialInterfaceProtocol):
    await twid.update_telem_sample_rate(20)
    
    for i in range(100):
        print(f'reboot cycle {i+1}/10')
        assert await twid.esp32_reboot()
    
    await twid.end()

async def reset_test(twid:TwidSerialInterfaceProtocol):
    await twid.update_telem_sample_rate(5)

    for i in range(100):
        print(f'reset cycle {i+1}/100')
        assert await twid.control_reset()
    
    await twid.end()

run_test(SERIAL_PORT, reset_test)
run_test(SERIAL_PORT, reboot_test)
