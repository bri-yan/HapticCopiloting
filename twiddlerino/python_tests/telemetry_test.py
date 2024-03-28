import unittest
import time
import asyncio
from serial_interface.serial_interface import TelemetryFrame
from serial_interface.serial_test import TwidBaseTest

class TelemetryTest(TwidBaseTest):

    async def test_telem_packet_loss(self):
        await asyncio.sleep(0.5) #wait for connection to init

        for rate in [200,100,50,20,10,1]:
            self.twid.control_reset()
            self.twid.update_telem_sample_rate(rate)
            await asyncio.sleep(1)
            self.twid.frame_count = 0
            start_sent = self.twid.last_frame.nframes_sent_serial
            start = time.time()
            while time.time() < start + 1:
                pass
            self.assertEqual(self.twid.frame_count, self.twid.last_frame.nframes_sent_serial-start_sent,'Telemetry frames were lost!')

if __name__ == "__main__":
    unittest.main()