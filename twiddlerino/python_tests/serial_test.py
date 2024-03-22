import random
import sys
import time
from collections import deque

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

async def run_test():
    global twid, loop
    transport, twid = await serial_asyncio.create_serial_connection(loop, TwidSerialInterfaceProtocol, SERIAL_PORT, baudrate=SERIAL_BAUD_RATE)
    await asyncio.sleep(0.5) #wait for connection to init
    twid.update_telem_sample_rate(10)
    await asyncio.sleep(0.5)
    
    while True:
        if not twid.frames.empty():
            latest:TelemetryFrame = twid.frames.get_nowait()
            print(f't:{latest.timestamp_ms}\tframes sent:{latest.nframes_sent_serial}\treceived:{twid.frame_count}')
            # print(latest)
        await asyncio.sleep(0.0001)

twid: TwidSerialInterfaceProtocol
loop = asyncio.get_event_loop()
loop.run_until_complete(run_test())
loop.close()

