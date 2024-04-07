import random
import sys
import time
from collections import deque

import numpy as np
import pygame

import asyncio
import serial_asyncio
from serial_interface.serial_interface import *

from game_objects import MassSpringDamper, Asteroid, EnemyProjectile, EnemyShip, PlayerShip, Path

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM9'

# async version of pygame.time.Clock
class AsyncClock:
    def __init__(self, time_func=pygame.time.get_ticks):
        self.time_func = time_func
        self.last_tick = time_func() or 0
 
    async def tick(self, fps=0):
        if 0 >= fps:
            return
 
        end_time = (1.0 / fps) * 1000
        current = self.time_func()
        time_diff = current - self.last_tick
        delay = (end_time - time_diff) / 1000
 
        self.last_tick = current
        if delay < 0:
            delay = 0
 
        await asyncio.sleep(delay)

async def run_game(game_interface:TwidSerialInterfaceProtocol):

    # Colors
    WHITE = (255, 255, 255)
    GRAY = (100, 100, 100)
    BLACK = (0, 0, 0)
    LIGHT_PURPLE = (205, 205, 250)
    ORANGE = (255, 69, 0)
    RED = (255, 0, 0)

    # Initialize Pygame
    pygame.init()

    # Set up the screen
    screen = pygame.display.set_mode((0,0), pygame.RESIZABLE)
    WIDTH, HEIGHT = screen.get_size()
    pygame.display.set_caption('Mass Spring Damper Demo')

    # Create game objects
    controls = [(pygame.K_a, pygame.K_d), (pygame.K_j, pygame.K_l)]
    clock = AsyncClock()

    msd_width, msd_height = 250, 250
    msd = MassSpringDamper(WIDTH//2 - msd_width//2, HEIGHT//2 - msd_height//2, msd_width, msd_height, 0, 0, 0)
    wall_width = 100
    wall = pygame.Rect(WIDTH - wall_width, 0, 100, HEIGHT)
    spring_width = wall.x - (msd.x + msd_width)
    spring_height = 100
    max_spring_width = 150

    color_factor = 0
    spring_color = (int(color_factor), int(255 - color_factor), 0)
    spring = pygame.Rect(msd.x + msd_width, msd.y + msd_height//3 - spring_height//2, spring_width, spring_height)

    damper_width, damper_height, damper_thickness = 50, 50, 10
    damper = pygame.Rect(msd.x + msd_width, msd.y + 4*msd_height//5 - damper_thickness//2, spring_width, damper_thickness)
    damper_block = pygame.Rect(damper.x + spring_width//2 - damper_width//2, damper.y + damper_thickness//2 - damper_width//2, damper_width, damper_height)

    await game_interface.update_control_type(TwidID.TWID1_ID, ControlType.POSITION_CTRL)
    await game_interface.update_pid(TwidID.TWID1_ID, Kp=1,Kd=0.1)
    

    # Main game loop
    
    while True:
        # Handle game events
        for event in pygame.event.get():
            # Handle game quit
            if event.type == pygame.QUIT or \
            (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                pygame.quit()
                sys.exit()
            
        # Clear the screen
        screen.fill(BLACK)

        # Draw game objects
        # Boundaries
        pygame.draw.line(screen, WHITE, (0, 0), (WIDTH, 0)) #top
        pygame.draw.line(screen, WHITE, (0, 0), (0, HEIGHT)) #left
        # pygame.draw.line(screen, WHITE, (WIDTH - 1, 0), (WIDTH - 1, HEIGHT), 200) #right
        pygame.draw.rect(screen, WHITE, wall)
        pygame.draw.rect(screen, spring_color, spring)            
        pygame.draw.rect(screen, WHITE, msd)
        pygame.draw.rect(screen, WHITE, damper)
        pygame.draw.rect(screen, WHITE, damper_block)

        

        # Update the display
        pygame.display.flip()

        #get the latest telemetry frame
        if not game_interface.frames_t1.empty():
            await game_interface.game_update_setpoint(TwidID.TWID1_ID, position=0.0)
            latest_frame: TelemetryFrame = game_interface.frames_t1.get_nowait()
            msd.x = latest_frame.position + WIDTH//2
            
            spring.x = msd.x + msd_width
            damper.x = msd.x + msd_width
            spring.width = wall.x - spring.x
            damper.width = wall.x - spring.x
            damper_block.x = damper.x + damper.width//2 - damper_width//2
            
            delta_x = msd.x - WIDTH//2 + 100 # 100 offset
            print(delta_x)
            color_factor = abs(255*(delta_x/max_spring_width))
            color_factor = max(0, min(255, color_factor))
            spring_color = (int(color_factor), int(255 - color_factor), 0)

        # Limit frames per second
        await clock.tick(45)

run_test(SERIAL_PORT, run_game)
