import random
import sys
import time
from collections import deque

import numpy as np

import asyncio
import serial_asyncio
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame
import socket

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000


SERIAL_STUDIO_HOST = '127.0.0.1'
SERIAL_STUDIO_PORT = 15555

async def run_test():
    global twid, loop
    transport, twid = await serial_asyncio.create_serial_connection(loop, TwidSerialInterfaceProtocol, SERIAL_PORT, baudrate=SERIAL_BAUD_RATE)
    await asyncio.sleep(0.5) #wait for connection to init
    twid.update_telem_sample_rate(1)
    await asyncio.sleep(0.5)
    
    while True:
        if not twid.frames.empty():
            latest:TelemetryFrame = twid.frames.get_nowait()
            print(f't:{latest.timestamp_ms}\tframes sent:{latest.nframes_sent_serial}\treceived:{twid.frame_count}')
            # print(latest)
        await asyncio.sleep(0.0001)

async def run_server():
    global twid, loop
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((SERIAL_STUDIO_HOST, SERIAL_STUDIO_PORT))
    server.listen(8)
    server.setblocking(False)
    client:socket.socket = None

    while True:
        if client is None:
            client, _ = await loop.sock_accept(server)
        if twid is not None:
            if not twid.socket_buffer.empty():
                data = twid.socket_buffer.get()
                try:
                    await loop.sock_sendall(client, data)
                except Exception as errmsg:
                    print(errmsg)
                    client.close()
                    client = None
        await asyncio.sleep(0.01)

twid: TwidSerialInterfaceProtocol = None
loop = asyncio.get_event_loop()
loop.run_until_complete(asyncio.gather(run_test(),run_server()))
loop.close()

