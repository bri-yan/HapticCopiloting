import random
import sys
import time
from collections import deque

import numpy as np
import pygame

import asyncio
from serial_interface.serial_interface import *

from game_objects import *
from game_config import *

game_interface: TwidSerialInterfaceProtocol

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

class SpaceshipDemo:
    def __init__(self):
        self.score = 0
        self.max_score = 0
        self.game_interface = None
        self.enable_pathview = True
        self.enable_tether = False
        self.K = 3e-2
        self.B = 1e-3
        self.J = 0.1
        self.error_history = deque(maxlen=100)
        self.asteroids = deque(maxlen=MAX_ASTEROIDS)
        self.projectiles = deque(maxlen=MAX_PROJECTILES)
        self.left_offset = -40
        self.right_offset = -40
        self.left_start = None
        self.right_start = None
    
    def reset_game(self):
        self.score = 0
        self.asteroids = deque(maxlen=MAX_ASTEROIDS)
        self.projectiles = deque(maxlen=MAX_PROJECTILES)

    async def error_based_stiffness(self, twid_id, error, k0, alpha):
        k = k0 + alpha*(error**2)
        # if twid_id == TwidID.TWID1_ID:
        #     print(str(twid_id) + ": " + str(k))
        await self.game_interface.update_impedance(twid_id, K=k, B=self.B, J=self.J)

    async def average_error_based_stiffness(self, twid_id, error, k0):
        max_error = 350
        min_error = 75
        error = abs(error)
        if error < min_error:
            error = 0
        if error > min_error:
            error = max_error
        self.error_history.append(error)
        median_error = np.median(self.error_history)
        average_error = np.mean(self.error_history)
        k = (1 - max(0, average_error/max_error)) * k0 + 1e-2
        if twid_id == TwidID.TWID1_ID:
            print(k)
        await self.game_interface.update_impedance(twid_id, K=k, B=self.B, J=self.J)

    async def run_game(self, game_interface: TwidSerialInterfaceProtocol):
        await game_interface.esp32_reboot()

        self.game_interface = game_interface
        # Initialize twiddlerinos
        await self.game_interface.update_control_type(TwidID.TWID1_ID, ControlType.IMPEDANCE_CTRL_SPRING)
        # await self.game_interface.update_pid(TwidID.TWID1_ID, Kp=2.5, Ki=0.0, Kd=0.1)
        await self.game_interface.update_impedance(TwidID.TWID1_ID, K=self.K, B=self.B, J=self.J)
        await self.game_interface.update_control_type(TwidID.TWID2_ID, ControlType.IMPEDANCE_CTRL_SPRING)
        # await self.game_interface.update_pid(TwidID.TWID2_ID, Kp=2.5, Ki=0.0, Kd=0.1)
        await self.game_interface.update_impedance(TwidID.TWID2_ID, K=self.K, B=self.B, J=self.J)
        await self.game_interface.update_telem_sample_rate(TwidID.TWID1_ID, 50)
        await self.game_interface.update_telem_sample_rate(TwidID.TWID2_ID, 50)
        # self.game_interface.game_update_impedance

        # Initialize Pygame
        pygame.init()
        clock = AsyncClock()

        # Set up the screen
        screen = pygame.display.set_mode((0,0), pygame.RESIZABLE)
        WIDTH, HEIGHT = screen.get_size()
        pygame.display.set_caption('Spaceships')
        font = pygame.font.Font(None, 36)  # Change the size as needed

        # Create game objects
        self.left_start, self.right_start = WIDTH//4, 3*WIDTH//4
        left_ship = PlayerShip(WIDTH//4 - SHIP_WIDTH//2, HEIGHT - 50 - SHIP_HEIGHT//2, SHIP_WIDTH, SHIP_HEIGHT, left_bound=0, right_bound=WIDTH//2)
        right_ship = PlayerShip(3*WIDTH//4 - SHIP_WIDTH//2, HEIGHT - 50 - SHIP_HEIGHT//2, SHIP_WIDTH, SHIP_HEIGHT, left_bound=WIDTH//2, right_bound=WIDTH)
        ships = [left_ship, right_ship]

        # Create events
        NEW_ASTEROID_EVENT = pygame.USEREVENT + 1
        # MAX_ASTEROIDS should be on screen at any given time
        pygame.time.set_timer(NEW_ASTEROID_EVENT, 50*(HEIGHT//ASTEROID_SPEED)//MAX_ASTEROIDS) 

        enemy_ship = EnemyShip(WIDTH//2 - SHIP_WIDTH//2, 10, SHIP_WIDTH, SHIP_HEIGHT, left_bound=0, right_bound=WIDTH)
        ENEMY_FIRE_EVENT = pygame.USEREVENT + 2
        pygame.time.set_timer(ENEMY_FIRE_EVENT, 1000)

        INCREMENT_SCORE_EVENT = pygame.USEREVENT + 3
        pygame.time.set_timer(INCREMENT_SCORE_EVENT, 1000)  # 1000 milliseconds = 1 second

        move_left = True
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

        # Main game loop
        while True:
            # Handle game events
            for event in pygame.event.get():
                # Handle game quit
                if event.type == pygame.QUIT or \
                (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    pygame.quit()
                    sys.exit()
                
                # Handle asteroid generation
                elif event.type == NEW_ASTEROID_EVENT:
                    x = random.randint(0 + ASTEROID_DIAMETER//2, WIDTH - ASTEROID_DIAMETER//2)
                    # print(index, xs[index])
                    # x = xs[index]
                    asteroid = Asteroid(x, 0, ASTEROID_DIAMETER, ASTEROID_DIAMETER, ASTEROID_SPEED)
                    # asteroid = Asteroid(x, 0, ASTEROID_DIAMETER, ASTEROID_DIAMETER, 1)
                    self.asteroids.append(asteroid)
                    # index += 1

                # Handle enemy fire generation
                elif event.type == ENEMY_FIRE_EVENT:
                    projectile = EnemyProjectile(enemy_ship.center_x - PROJECTILE_WIDTH//2, enemy_ship.y + enemy_ship.height, PROJECTILE_WIDTH, PROJECTILE_HEIGHT, PROJECTILE_SPEED )
                    self.projectiles.append(projectile)

                elif event.type == INCREMENT_SCORE_EVENT:
                    self.score += 10

            # Handle asteroid collision
            for asteroid in self.asteroids:
                for ship in ships:
                    if asteroid.colliderect(ship):
                        self.reset_game()

            # Handle projectile collision
            for projectile in self.projectiles:
                for ship in ships:
                    if projectile.colliderect(ship):
                        self.reset_game()
                
            # Clear the screen
            screen.fill(BLACK)

            # Draw game objects
            # Score
            self.max_score = max(self.score, self.max_score)
            score_text = font.render(f'Score: {self.score}', True, (255, 255, 255))  # Change the color as needed
            screen.blit(score_text, (10, 10))  # Change the position as needed
            max_score_text = font.render(f'Max Score: {self.max_score}', True, (255, 255, 255))  # Change the color as needed
            screen.blit(max_score_text, (10, 40))  # Change the position as needed
            
            # Boundaries
            pygame.draw.line(screen, WHITE, (0, 0), (WIDTH, 0))
            pygame.draw.line(screen, WHITE, (0, 0), (0, HEIGHT))
            pygame.draw.line(screen, WHITE, (WIDTH - 1, 0), (WIDTH - 1, HEIGHT))
            pygame.draw.line(screen, GRAY, (WIDTH//2 - 1, 0), (WIDTH//2 - 1, HEIGHT), 2)

            # Tether
            if self.enable_tether:
                tether_length = max(0, min(MAX_TETHER_LENGTH, abs(right_ship.center_x - left_ship.center_x)))
                color_factor = 255*(tether_length/MAX_TETHER_LENGTH)**2
                TETHER_COLOR = (int(color_factor), int(255 - color_factor), 0)
                pygame.draw.line(screen, TETHER_COLOR, (left_ship.center_x, left_ship.center_y), (right_ship.center_x, right_ship.center_y), 5)

            # Player ships
            for ship in ships:
                pygame.draw.polygon(screen, ORANGE, ship.engine_vertices)
                pygame.draw.polygon(screen, LIGHT_PURPLE, ship.body_vertices)
                ship.switch = not ship.switch # basic ship animation

            # Asteroids
            for asteroid in self.asteroids:
                pygame.draw.ellipse(screen, WHITE, asteroid)
                asteroid.move()
            
            # Projectiles
            for projectile in self.projectiles:
                pygame.draw.rect(screen, RED, projectile)
                projectile.move()
            
            # Enemy ship
            pygame.draw.polygon(screen, ORANGE, enemy_ship.engine_vertices)
            pygame.draw.polygon(screen, LIGHT_PURPLE, enemy_ship.body_vertices)

            # Adjust paths
            left_path.adjust(left_ship.x, self.asteroids, 50, screen)
            right_path.adjust(right_ship.x, self.asteroids, -50, screen)

            # Handle enemy movement
            handle_enemy_movement()

            # Handle toggles
            keys = pygame.key.get_pressed()

            # Tether toggle
            if keys[pygame.K_t]:
                self.enable_tether = not self.enable_tether

            # Pathview toggle
            if keys[pygame.K_p]:
                self.enable_pathview = not self.enable_pathview
            if self.enable_pathview:
                for path in paths:
                    path.draw(screen)

            # Offset adjustment
            if keys[pygame.K_a]:
                self.left_offset -= 1
            if keys[pygame.K_d]:
                self.left_offset += 1
            if keys[pygame.K_j]:
                self.right_offset -= 1
            if keys[pygame.K_l]:
                self.right_offset += 1

            # Update the display
            pygame.display.flip()

            #get the latest telemetry frame
            if not self.game_interface.frames_t1.empty():
                target_position = paths[1].get_target(12, screen) - self.right_start
                await self.game_interface.game_update_setpoint(TwidID.TWID1_ID, position=target_position)
                latest_frame: TelemetryFrame = self.game_interface.latest_frame_t1
                right_ship.x = latest_frame.position + self.right_offset + self.right_start
                error = target_position - latest_frame.position
                # await self.error_based_stiffness(TwidID.TWID1_ID, error, 3e-2, 1e-6)
                await self.average_error_based_stiffness(TwidID.TWID1_ID, error, 1e-1)
                # print(right_ship.x, target_position)
                # right_ship.x = latest_frame.position - right_ship.width//2
                # print(f'frame #: {self.game_interface.frame_count}, frame: {latest_frame}')

            if not self.game_interface.frames_t2.empty():
                target_position = paths[0].get_target(12, screen) - self.left_start
                await self.game_interface.game_update_setpoint(TwidID.TWID2_ID, position=target_position)
                latest_frame: TelemetryFrame = self.game_interface.latest_frame_t2
                left_ship.x = latest_frame.position + self.left_offset + self.left_start
                # await self.error_based_stiffness(TwidID.TWID2_ID, error, 1e-2, 1e-6)
                error = target_position - latest_frame.position
                # await self.average_error_based_stiffness(TwidID.TWID2_ID, error, 1e-1)
                # print(left_ship.x, target_position)

            # Limit frames per second
            await clock.tick(60)

# run_test(SERIAL_PORT, run_game)
