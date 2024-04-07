from game import SpaceshipDemo
from serial_interface.serial_interface import *
SERIAL_PORT = 'COM5'

demo = SpaceshipDemo()
run_test(SERIAL_PORT, demo.run_game)
