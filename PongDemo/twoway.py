import pygame
import serial
import sys
from collections import deque
import numpy as np
from PID import PID
import time

# Configure port
serial_inst = serial.Serial()
port = "COM3"

serial_inst.baudrate = 115200
serial_inst.port = port 
serial_inst.timeout = 1
serial_inst.open()
time.sleep(2)

def write_command(command):
    try:
        serial_inst.write(command.encode())
        print(f'Sent command: {command}')
    except Exception as e:
        print(f"Error sending command: {e}")

# Read from serial port
buffer = deque(maxlen=10)
def read_position():
    try:
        write_command('r')

        data = serial_inst.read(4)
        # Read sign bit, low byte, and high byte separately
        # sign_bit = ord(serial_inst.read(1))
        # low_byte = ord(serial_inst.read(1))
        # high_byte = ord(serial_inst.read(1))
        ack_byte, sign_bit, low_byte, high_byte = data

        if ack_byte != b'a':
            print("Error: Acknowledgement byte not received")
            return -1

        # Reconstruct position value with sign
        position = (high_byte << 8) + low_byte
        if sign_bit:
            position = -position
        
        buffer.append(position)
        smoothed_position = np.median(buffer)

        # Optionally, skip previous position values
        serial_inst.flushInput()

        return -int(smoothed_position)
    except Exception as e:
        print("Error:", e)
        return str(e)
    
def write_target(target):
    command = f'w{target}'
    write_command(command)
    time.sleep(0.5)
    ack_byte = serial_inst.read(1)
    if ack_byte != b'a':
        print("Error: Acknowledgement byte not received")
        print(ack_byte)
        return -1
    return 0


# Initialize Pygame
pygame.init()

# Set up the screen
screen = pygame.display.set_mode((400,400), pygame.RESIZABLE)
WIDTH, HEIGHT = screen.get_size()
pygame.display.set_caption('Pong')

clock = pygame.time.Clock()

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or \
        (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            pygame.quit()
            sys.exit()
        elif (event.type == pygame.KEYDOWN and event.key == pygame.K_r):
            print(int(read_position()))
        elif (event.type == pygame.KEYDOWN and event.key == pygame.K_w):
            write_target(1000)
        elif (event.type == pygame.KEYDOWN and event.key == pygame.K_e):
            write_target(0)

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)
