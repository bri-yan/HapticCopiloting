import time
from collections import deque
import numpy as np
import serial


class SerialCommunication:
    def __init__(self, port="COM3", baudrate=115200):
        self.buffer = deque(maxlen=10)
        self.serial_inst = serial.Serial()
        self.serial_inst.baudrate = baudrate
        self.serial_inst.port = port 
        self.serial_inst.open()
        time.sleep(3)

    def write_command(self, command):
        try:
            self.serial_inst.write(command.encode())
            if command[0] == 'w':
                print(f'Sent command: {command}')
        except Exception as e:
            print(f"Error sending command: {e}")

    # Read from serial port
    def read_position(self):
        try:
            self.write_command('r')

            data = self.serial_inst.read(4)
            # Read sign bit, low byte, and high byte separately
            # sign_bit = ord(self.serial_inst.read(1))
            # low_byte = ord(self.serial_inst.read(1))
            # high_byte = ord(self.serial_inst.read(1))
            ack_byte, sign_bit, low_byte, high_byte = data

            if ack_byte != ord('a'):
                print("Error: Acknowledgement byte not received")
                return -1

            # Reconstruct position value with sign
            position = (high_byte << 8) + low_byte
            if sign_bit:
                position = -position
            
            self.buffer.append(position)
            smoothed_position = np.median(self.buffer)

            # Optionally, skip previous position values
            # self.serial_inst.flushInput()
            # print(-int(smoothed_position))
            return -int(smoothed_position)
        except Exception as e:
            print("Error:", e)
            return str(e)
        
    def write_target(self, target):
        command = f'w{target}'
        self.write_command(command)
        ack_byte = self.serial_inst.read(1)
        if ack_byte != b'a':
            print("Error: Acknowledgement byte not received")
            print(ack_byte)
            return -1
        return 0