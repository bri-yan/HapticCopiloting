from unittest import IsolatedAsyncioTestCase
import unittest
from serial_interface.serial_interface import TwidSerialInterfaceProtocol, TelemetryFrame, run_socket_server_async, run_socket_server_thread
import serial_asyncio, asyncio
import threading

SERIAL_PORT = 'COM9'
SERIAL_BAUD_RATE = 1000000

class TwidBaseTest(IsolatedAsyncioTestCase):
    
    def setUp(self) -> None:
        super().setUp()
        self.socket_thread = threading.Thread(target=lambda: run_socket_server_thread(self))
        self.socket_thread.start()

    async def asyncSetUp(self):
        loop = asyncio.get_event_loop()
        _, t = await serial_asyncio.create_serial_connection(loop, TwidSerialInterfaceProtocol, SERIAL_PORT, baudrate=SERIAL_BAUD_RATE)
        self.twid:TwidSerialInterfaceProtocol = t
        await asyncio.sleep(0.5)
        self.twid.control_reset()

    async def asyncTearDown(self):
        self.twid.motor_stop()
        self.twid.transport.close()
        
    def tearDown(self) -> None:
        self.socket_thread.join(10)