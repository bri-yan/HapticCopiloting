import pygame
import serial
import sys
from collections import deque
import numpy as np
from PID import PID
import time
import random

# Configure port
serial_inst = serial.Serial()
port = "COM3"
serial_inst.baudrate = 115200
serial_inst.port = port 
# TODO: uncomment to establish serial
# serial_inst.open()
# time.sleep(3)

def write_command(command):
    try:
        serial_inst.write(command.encode())
        if command[0] == 'w':
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

        if ack_byte != ord('a'):
            print("Error: Acknowledgement byte not received")
            return -1

        # Reconstruct position value with sign
        position = (high_byte << 8) + low_byte
        if sign_bit:
            position = -position
        
        buffer.append(position)
        smoothed_position = np.median(buffer)

        # Optionally, skip previous position values
        # serial_inst.flushInput()
        # print(-int(smoothed_position))
        return -int(smoothed_position)
    except Exception as e:
        print("Error:", e)
        return str(e)
    
def write_target(target):
    command = f'w{target}'
    write_command(command)
    ack_byte = serial_inst.read(1)
    if ack_byte != b'a':
        print("Error: Acknowledgement byte not received")
        print(ack_byte)
        return -1
    return 0

# Colors
WHITE = (255, 255, 255)
GRAY = (50, 50, 50)
BLACK = (0, 0, 0)
LIGHT_PURPLE = (205, 205, 250)
ORANGE = (255, 69, 0)
RED = (255, 0, 0)

# Classes
class PlayerShip(pygame.Rect):
    def __init__(self, x, y, width, height, left_bound, right_bound):
        super().__init__(x, y, width, height)
        self.left_bound, self.right_bound = left_bound, right_bound

        # Physical properties
        self.thrust = 5
        self.damping = 0.465
        self.inertia = 0.5
        self.velocity = 0
        self.acceleration = 0
        self.mu_s, self.mu_k = 1, 2
        self.switch = True
        
    def move(self):
        self.enforce_bounds()
        self.velocity += self.acceleration
        # self.velocity = max(-10, min(10, self.velocity))  # Limit the velocity to a range
        self.friction()
        self.x += self.velocity
        self.acceleration = 0 # reset acceleration calculation for next frame

    def accelerate(self, force): # Exert a force on the spaceship
        self.acceleration += force / self.inertia

    def thrust_right(self):
        self.accelerate(self.thrust)
    
    def thrust_left(self):
        self.accelerate(-self.thrust)
        
    def friction(self):
        # static friction
        if self.acceleration < self.mu_s and self.velocity == 0:
            self.acceleration = 0
        # kinetic friction
        else:
            self.accelerate(-np.sign(self.velocity)*min(self.mu_k, self.inertia*self.acceleration))
        # damping
        self.velocity *= (1 - self.damping)
          
    def enforce_bounds(self):
        error = 0
        if self.x <= self.left_bound + self.width//2:
            error = self.left_bound + self.width//2 - self.x
            self.velocity *= (1 - self.damping)
        if self.x + self.width >= self.right_bound - self.width//2:
            error = self.right_bound - self.width//2 - (self.x + self.width)
            self.velocity *= (1 - self.damping)
        self.accelerate(0.1*error)
    
    @property
    def center_x(self):
        return self.x + self.width//2
    
    @property
    def center_y(self):
        return self.y + self.height//2

    @property
    def body_vertices(self):
        return [(self.x, self.y + self.height),
                (self.center_x, self.y + 4*self.height//5),
                (self.x + self.width, self.y + self.height),
                (self.center_x, self.y)]
    
    @property
    def engine_vertices(self):
        long = [(self.center_x - self.width//5, self.y + 4*self.height//5), 
                (self.center_x + self.width//5, self.y + 4*self.height//5), 
                (self.center_x, self.y + self.height)]
        short = [(self.center_x - self.width//5, self.y + 4*self.height//5), 
                (self.center_x + self.width//5, self.y + 4*self.height//5), 
                (self.center_x, self.y + self.height - self.height//10)]
        return long if self.switch else short

class Asteroid(pygame.Rect):
    def __init__(self, x, y, width, height, velocity):
        super().__init__(x, y, width, height)
        self.velocity = velocity
    
    def move(self):
        self.y += self.velocity

class EnemyShip(pygame.Rect):
    def __init__(self, x, y, width, height, left_bound, right_bound):
        super().__init__(x, y, width, height)
        self.left_bound, self.right_bound = left_bound, right_bound

        # Physical properties
        self.thrust = 1
        self.damping = 0.465
        self.inertia = 0.5
        self.velocity = 0
        self.acceleration = 0
        self.mu_s, self.mu_k = 1, 2
        self.switch = True
        
    def move(self):
        self.enforce_bounds()
        self.velocity += self.acceleration
        # self.velocity = max(-10, min(10, self.velocity))  # Limit the velocity to a range
        self.friction()
        self.x += self.velocity
        self.acceleration = 0 # reset acceleration calculation for next frame

    def accelerate(self, force): # Exert a force on the spaceship
        self.acceleration += force / self.inertia

    def thrust_right(self):
        self.accelerate(self.thrust)
    
    def thrust_left(self):
        self.accelerate(-self.thrust)
        
    def friction(self):
        # static friction
        if self.acceleration < self.mu_s and self.velocity == 0:
            self.acceleration = 0
        # kinetic friction
        else:
            self.accelerate(-np.sign(self.velocity)*min(self.mu_k, self.inertia*self.acceleration))
        # damping
        self.velocity *= (1 - self.damping)
          
    def enforce_bounds(self):
        error = 0
        if self.x <= self.left_bound + self.width//2:
            error = self.left_bound + self.width//2 - self.x
            self.velocity *= (1 - self.damping)
        if self.x >= self.right_bound- self.width//2:
            error = self.right_bound - self.width//2 - self.x
            self.velocity *= (1 - self.damping)
        self.accelerate(0.1*error)
    
    @property
    def center_x(self):
        return self.x + self.width//2
    
    @property
    def center_y(self):
        return self.y + self.height//2

    @property
    def body_vertices(self):
        return [(self.x, self.y),
                (self.center_x, self.y + self.height//5),
                (self.x + self.width, self.y),
                (self.center_x, self.y + self.height)]
    
    @property
    def engine_vertices(self):
        long = [(self.center_x - self.width//5, self.y + self.height//5), 
                (self.center_x + self.width//5, self.y + self.height//5), 
                (self.center_x, self.y)]
        short = [(self.center_x - self.width//5, self.y + self.height//5), 
                (self.center_x + self.width//5, self.y + self.height//5), 
                (self.center_x, self.y + self.height//10)]
        return long if self.switch else short

class EnemyProjectile(pygame.Rect):
    def __init__(self, x, y, width, height, velocity):
        super().__init__(x, y, width, height)
        self.velocity = velocity
    
    def move(self):
        self.y += self.velocity


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
clock = pygame.time.Clock()

# Additional setup
MIDDLE = HEIGHT // 2
POSITION_SCALING_FACTOR = 0.8

tether_k = 1.5e-5
tether_color = (200, 255, 0)
max_tether_length = 550
tether_equilibrium = 150

asteroid_diameter = WIDTH // 15
max_asteroids = 3
asteroid_speed = 3
asteroids = deque(maxlen=max_asteroids)
NEW_ASTEROID_EVENT = pygame.USEREVENT + 1
# max_asteroids should be on screen at any given time
pygame.time.set_timer(NEW_ASTEROID_EVENT, 30*(HEIGHT//asteroid_speed)//max_asteroids) 

enemy_ship = EnemyShip(WIDTH//2 - ship_width//2, 10, ship_width, ship_height, left_bound=0, right_bound=WIDTH)
move_left = True
projectile_width, projectile_height = 3, 8
max_projectiles = 10
projectile_speed = 5
projectiles = deque(maxlen=max_projectiles)
ENEMY_FIRE_EVENT = pygame.USEREVENT + 2
pygame.time.set_timer(ENEMY_FIRE_EVENT, 750)

def handle_enemy_movement():
    global move_left
    if move_left:
        if enemy_ship.center_x < enemy_ship.width:
            move_left = False
        enemy_ship.thrust_left()
    else:
        if enemy_ship.center_x > WIDTH - enemy_ship.width:
            move_left = True
        enemy_ship.thrust_right()
    enemy_ship.move()
        

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
            x = random.randint(0 + asteroid_diameter//2, WIDTH - asteroid_diameter//2)
            y = 0
            asteroid = Asteroid(x, y, asteroid_diameter, asteroid_diameter, asteroid_speed)
            asteroids.append(asteroid)

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
    
    # Handle enemy movement
    handle_enemy_movement()

    # Handle player movement
    keys = pygame.key.get_pressed()
    for idx, ship in enumerate(ships):
        left, right = keys[controls[idx][0]], keys[controls[idx][1]]
        if left or right:
            if right:
                ship.thrust_right() # Accelerate to the right
            if left:
                ship.thrust_left()
        
    # Clear the screen
    screen.fill(BLACK)

    # Handle tether properties
    tether_length = max(0, min(max_tether_length, abs(right_ship.center_x - left_ship.center_x)))
    color_factor = 255*(tether_length/max_tether_length)**2
    tether_color = (int(color_factor), int(255 - color_factor), 0)

    # Draw game objects
    # Boundaries
    pygame.draw.line(screen, WHITE, (0, 0), (0, HEIGHT))
    pygame.draw.line(screen, WHITE, (WIDTH - 1, 0), (WIDTH - 1, HEIGHT))
    pygame.draw.line(screen, GRAY, (WIDTH//2 - 1, 0), (WIDTH//2 - 1, HEIGHT), 2)

    pygame.draw.line(screen, tether_color, (left_ship.center_x, left_ship.center_y), (right_ship.center_x, right_ship.center_y), 5)

    for ship in ships:
        pygame.draw.polygon(screen, ORANGE, ship.engine_vertices)
        pygame.draw.polygon(screen, LIGHT_PURPLE, ship.body_vertices)
    
        ship.switch = not ship.switch
        error = (right_ship.x - left_ship.x - tether_equilibrium)
        error = error*abs(error) # allows for tether to push ships apart
        left_ship.accelerate(tether_k*error)
        right_ship.accelerate(-tether_k*error)
        ship.move()

    for asteroid in asteroids:
        pygame.draw.ellipse(screen, WHITE, asteroid)
        asteroid.move()
    
    for projectile in projectiles:
        pygame.draw.rect(screen, RED, projectile)
        projectile.move()
        
    pygame.draw.polygon(screen, ORANGE, enemy_ship.engine_vertices)
    pygame.draw.polygon(screen, LIGHT_PURPLE, enemy_ship.body_vertices)
                

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)