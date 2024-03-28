import random
import sys
import time
from collections import deque

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_socket_server_async
import socket

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

async def run_test():
    global loop
    twid:TwidSerialInterfaceProtocol = None
    transport, twid = await serial_asyncio.create_serial_connection(loop, TwidSerialInterfaceProtocol, SERIAL_PORT, baudrate=SERIAL_BAUD_RATE)
    await asyncio.sleep(0.5) #wait for connection to init
    twid.update_telem_sample_rate(200)
    await asyncio.sleep(0.5)
    p = 0
    while True:
        twid.update_setpoint(position=p)
        p+=1
        if not twid.frames.empty():
            latest:TelemetryFrame = twid.frames.get_nowait()
            print(latest)
        await asyncio.sleep(0.01)

loop = asyncio.get_event_loop()
loop.run_until_complete(asyncio.gather(run_test(), run_socket_server_async()))
loop.close()

