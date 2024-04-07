import random
import sys
import time
from collections import deque

import numpy as np
import pygame

import asyncio
from serial_interface.serial_interface import *

from game_objects import Asteroid, EnemyProjectile, EnemyShip, PlayerShip, Path

game_interface: TwidSerialInterfaceProtocol

###SERIAL CONFIGURATION for esp32
SERIAL_PORT = 'COM5'

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
    GRAY = (50, 50, 50)
    BLACK = (0, 0, 0)
    LIGHT_PURPLE = (205, 205, 250)
    ORANGE = (255, 69, 0)
    RED = (255, 0, 0)

    # Initialize Pygame
    pygame.init()

    # Set up the screen
    screen = pygame.display.set_mode((0,0), pygame.RESIZABLE)
    WIDTH, HEIGHT = screen.get_size()
    pygame.display.set_caption('Spaceships')

    # Create game objects
    ship_width, ship_height = 68, 60
    left_ship = PlayerShip(WIDTH//4 - ship_width//2, HEIGHT - 50 - ship_height//2, ship_width, ship_height, left_bound=0, right_bound=WIDTH//2)
    right_ship = PlayerShip(3*WIDTH//4 - ship_width//2, HEIGHT - 50 - ship_height//2, ship_width, ship_height, left_bound=WIDTH//2, right_bound=WIDTH)
    ships = [left_ship, right_ship]
    controls = [(pygame.K_a, pygame.K_d), (pygame.K_j, pygame.K_l)]
    clock = AsyncClock()

    # Additional setup
    MIDDLE = HEIGHT // 2
    POSITION_SCALING_FACTOR = 0.8

    tether_k = 1.5e-5
    tether_color = (200, 255, 0)
    max_tether_length = 900
    tether_equilibrium = 500

    asteroid_diameter = WIDTH // 15
    max_asteroids = 10
    asteroid_speed = 3
    asteroids = deque(maxlen=max_asteroids)
    NEW_ASTEROID_EVENT = pygame.USEREVENT + 1
    # max_asteroids should be on screen at any given time
    pygame.time.set_timer(NEW_ASTEROID_EVENT, 50*(HEIGHT//asteroid_speed)//max_asteroids) 

    enemy_ship = EnemyShip(WIDTH//2 - ship_width//2, 10, ship_width, ship_height, left_bound=0, right_bound=WIDTH)
    move_left = True
    projectile_width, projectile_height = 10, 20
    max_projectiles = 10
    projectile_speed = 5
    projectiles = deque(maxlen=max_projectiles)
    ENEMY_FIRE_EVENT = pygame.USEREVENT + 2
    pygame.time.set_timer(ENEMY_FIRE_EVENT, 1000)

    def handle_enemy_movement():
        nonlocal move_left
        if move_left:
            if enemy_ship.center_x < enemy_ship.width:
                move_left = False
            enemy_ship.thrust_left()
        else:
            if enemy_ship.center_x > WIDTH - enemy_ship.width:
                move_left = True
            enemy_ship.thrust_right()
        enemy_ship.move()
            
    left_path = Path(left_ship.width, left_ship.height, WIDTH//4 - left_ship.width//2, HEIGHT, -left_ship.height//2, 100)
    right_path = Path(right_ship.width, right_ship.height, 3*WIDTH//4 - right_ship.width//2, HEIGHT, -right_ship.height//2, 100)
    paths = [left_path, right_path]
    index = 0
    xs = [3*WIDTH//4, 3*WIDTH//4 - 50, 3*WIDTH//4 - 100]
    print(3*WIDTH//4)
    
    # asteroids = [
    #     Asteroid(3*WIDTH//4, HEIGHT//2, asteroid_diameter, asteroid_diameter, asteroid_speed),
    #     Asteroid(3*WIDTH//4 - 100, HEIGHT//2 - 100, asteroid_diameter, asteroid_diameter, asteroid_speed),
    # ]

    await game_interface.update_control_type(TwidID.TWID1_ID, ControlType.POSITION_CTRL)
    await game_interface.update_pid(TwidID.TWID1_ID, Kp=2.5, Ki=0.0, Kd=0.1)
    await game_interface.update_control_type(TwidID.TWID2_ID, ControlType.POSITION_CTRL)
    await game_interface.update_pid(TwidID.TWID2_ID, Kp=2.5, Ki=0.0, Kd=0.1)
    show_path = True
    
    # Main game loop
    while True:
        # Handle game events
        index = index % len(xs)
        for event in pygame.event.get():
            # Handle game quit
            if event.type == pygame.QUIT or \
            (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                pygame.quit()
                sys.exit()
            
            # Handle asteroid generation
            elif event.type == NEW_ASTEROID_EVENT:
                x = random.randint(0 + asteroid_diameter//2, WIDTH - asteroid_diameter//2)
                # print(index, xs[index])
                # x = xs[index]
                asteroid = Asteroid(x, 0, asteroid_diameter, asteroid_diameter, asteroid_speed)
                # asteroid = Asteroid(x, 0, asteroid_diameter, asteroid_diameter, 1)
                asteroids.append(asteroid)
                # index += 1

            # Handle enemy fire
            elif event.type == ENEMY_FIRE_EVENT:
                projectile = EnemyProjectile(enemy_ship.center_x - projectile_width//2, enemy_ship.y + enemy_ship.height, projectile_width, projectile_height, projectile_speed)
                projectiles.append(projectile)

        # Handle asteroid collision
        for asteroid in asteroids:
            for ship in ships:
                if asteroid.colliderect(ship):
                    projectiles = deque(maxlen=max_projectiles)
                    asteroids = deque(maxlen=max_asteroids)

        # Handle projectile collision
        for projectile in projectiles:
            for ship in ships:
                if projectile.colliderect(ship):
                    projectiles = deque(maxlen=max_projectiles)
                    asteroids = deque(maxlen=max_asteroids)
            
        # Clear the screen
        screen.fill(BLACK)

        # Handle tether properties
        tether_length = max(0, min(max_tether_length, abs(right_ship.center_x - left_ship.center_x)))
        color_factor = 255*(tether_length/max_tether_length)**2
        tether_color = (int(color_factor), int(255 - color_factor), 0)

        # Draw game objects
        # Boundaries
        pygame.draw.line(screen, WHITE, (0, 0), (WIDTH, 0))
        pygame.draw.line(screen, WHITE, (0, 0), (0, HEIGHT))
        pygame.draw.line(screen, WHITE, (WIDTH - 1, 0), (WIDTH - 1, HEIGHT))
        pygame.draw.line(screen, GRAY, (WIDTH//2 - 1, 0), (WIDTH//2 - 1, HEIGHT), 2)

        pygame.draw.line(screen, tether_color, (left_ship.center_x, left_ship.center_y), (right_ship.center_x, right_ship.center_y), 5)

        for ship in ships:
            pygame.draw.polygon(screen, ORANGE, ship.engine_vertices)
            pygame.draw.polygon(screen, LIGHT_PURPLE, ship.body_vertices)
        
            ship.switch = not ship.switch
            # delta = (right_ship.x - left_ship.x - tether_equilibrium)
            # # delta = delta*abs(delta) # allows for tether to push ships apart
            # left_ship.accelerate(tether_k*delta)
            # right_ship.accelerate(-tether_k*delta)
            ship.move()

        for asteroid in asteroids:
            pygame.draw.ellipse(screen, WHITE, asteroid)
            asteroid.move()
        
        for projectile in projectiles:
            pygame.draw.rect(screen, RED, projectile)
            projectile.move()
            
        pygame.draw.polygon(screen, ORANGE, enemy_ship.engine_vertices)
        pygame.draw.polygon(screen, LIGHT_PURPLE, enemy_ship.body_vertices)

        left_path.adjust(left_ship.x, asteroids, 50, screen)
        right_path.adjust(right_ship.x, asteroids, -50, screen)

        # Handle enemy movement
        handle_enemy_movement()

        # Handle player movement
        keys = pygame.key.get_pressed()
        # for idx, ship in enumerate(ships[:1]): # TODO: remove slice
        #     left, right = keys[controls[idx][0]], keys[controls[idx][1]]
        #     if left or right:
        #         if right:
        #             ship.thrust_right() # Accelerate to the right
        #         if left:
        #             ship.thrust_left()

        if keys[pygame.K_t]:
            show_path = not show_path
        if show_path:
            for path in paths:
                for idx, breadcrumb in enumerate(path.breadcrumbs):
                    crumb = pygame.Rect(breadcrumb.center_x, breadcrumb.center_y, 2, 2)
                    if idx % 2 == 0:
                        pygame.draw.rect(screen, WHITE, crumb)
                    else:
                        pygame.draw.rect(screen, WHITE, crumb)
        
        # Handle pathing
        # for idx, ship in enumerate(ships[:1]): # TODO: remove slice
        #     left, right = keys[controls[idx][0]], keys[controls[idx][1]]
        #     if not left and not right:
        #         ship.pid_control(paths[idx].get_target(9, screen))
        

        # Update the display
        pygame.display.flip()

        #get the latest telemetry frame
        if not game_interface.frames_t1.empty():
            offset = -40
            target_position = paths[1].get_target(14, screen)
            await game_interface.game_update_setpoint(TwidID.TWID1_ID, position=target_position)
            latest_frame: TelemetryFrame = game_interface.frames_t1.get_nowait()
            right_ship.x = latest_frame.position + offset
            print(right_ship.x, target_position)
            # right_ship.x = latest_frame.position - right_ship.width//2
            # print(f'frame #: {game_interface.frame_count}, frame: {latest_frame}')

        if not game_interface.frames_t2.empty():
            offset = -40
            target_position = paths[0].get_target(14, screen)
            await game_interface.game_update_setpoint(TwidID.TWID2_ID, position=target_position)
            latest_frame: TelemetryFrame = game_interface.frames_t2.get_nowait()
            left_ship.x = latest_frame.position + offset
            print(left_ship.x, target_position)

        # Limit frames per second
        await clock.tick(45)

run_test(SERIAL_PORT, run_game)
