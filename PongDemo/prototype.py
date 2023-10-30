import pygame
import sys

# Constants
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

class Paddle(pygame.Rect):
    def __init__(self, x, y):
        super().__init__(x, y, 20, 100)

    def move(self, dy):
        self.y += dy
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

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or \
        (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            pygame.quit()
            sys.exit()

    # Player controls
    keys = pygame.key.get_pressed()
    velocity = 5*(keys[pygame.K_DOWN] - keys[pygame.K_UP])
    player_paddle.move(velocity)

    # Computer AI (simple tracking of the ball)
    if ball.centery < computer_paddle.centery:
        computer_paddle.move(-5)
    elif ball.centery > computer_paddle.centery:
        computer_paddle.move(5)

    # Move the ball
    ball.move()
    ball.check_collision(player_paddle)
    ball.check_collision(computer_paddle)

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
