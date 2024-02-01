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
serial_inst.open()

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

# Constants
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

class Paddle(pygame.Rect):
    def __init__(self, x, y):
        super().__init__(x, y, 20, 100)

    def move(self, dy):
        self.y += dy
        self.y = max(0, min(HEIGHT - self.height, self.y))
    
    def move_pos(self, y):
        self.y = y
        self.y = max(0, min(HEIGHT - self.height, self.y))

class Ball(pygame.Rect):
    def __init__(self, x, y, vx, vy):
        super().__init__(x, y, 15, 15)
        self.vx = vx
        self.vy = vy

    def move(self):
        self.x += self.vx
        self.y += self.vy

        # Ball collision with top and bottom
        if self.top <= 0 or self.bottom >= HEIGHT:
            self.vy *= -1

    def check_collision(self, paddle):
        if self.colliderect(paddle):
            self.vx *= -1
            return True
        else:
            return False

# Initialize Pygame
pygame.init()

# Set up the screen
screen = pygame.display.set_mode((0,0), pygame.FULLSCREEN)
WIDTH, HEIGHT = screen.get_size()
pygame.display.set_caption('Pong')

# Create game objects
player_paddle = Paddle(20, HEIGHT // 2 - 50)
computer_paddle = Paddle(WIDTH - 40, HEIGHT // 2 - 50)
ball = Ball(WIDTH // 2, HEIGHT // 2, 5, 5)
clock = pygame.time.Clock()

# Additional setup
MIDDLE = HEIGHT // 2
POSITION_SCALING_FACTOR = 0.8
prev_position = MIDDLE
buffer = deque(maxlen=10)
computer_control = PID(kp=1.8, ki=0, kd=0.1)
player_control = PID(kp=1.5, ki=0, kd=0.1)
player_score, computer_score = 0, 0
target = 100

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or \
        (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            pygame.quit()
            sys.exit()

    # Check for game end
    if ball.left <= 0 or ball.right >= WIDTH:
        if ball.left <= 0:
            computer_score += 1
        elif ball.right >= WIDTH:
            player_score += 1
        print("Player:", player_score, "Computer:", computer_score)
        ball = Ball(WIDTH // 2, HEIGHT // 2, 5, 5)
        computer_paddle.move_pos(HEIGHT // 2 - 50)
        continue

    # Player controls with arduino
    target = read_position(buffer)*POSITION_SCALING_FACTOR + MIDDLE
    player_paddle.move(target - prev_position)
    prev_position = target

    # Computer controls
    speed = computer_control.output(target=ball.centery, current=computer_paddle.centery)
    computer_paddle.move(speed)

    # Move the ball
    ball.move()
    if ball.check_collision(player_paddle) or ball.check_collision(computer_paddle):
        ball.vx *= 1.1
        ball.vy *= 1.1

        if ball.check_collision(player_paddle):
            write_target(-target)
            target = -target


    # Clear the screen
    screen.fill(BLACK)

    # Draw game objects
    pygame.draw.rect(screen, WHITE, player_paddle)
    pygame.draw.rect(screen, WHITE, computer_paddle)
    pygame.draw.ellipse(screen, WHITE, ball)

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)