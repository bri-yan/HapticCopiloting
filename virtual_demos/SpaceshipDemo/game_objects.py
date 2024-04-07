import numpy as np
import pygame

from game_config import *

class Asteroid(pygame.Rect):
    def __init__(self, x, y, width, height, velocity):
        super().__init__(x, y, width, height)
        self.velocity = velocity
    
    def move(self):
        self.y += self.velocity

class PlayerShip(pygame.Rect):
    def __init__(self, x, y, width, height, left_bound, right_bound):
        super().__init__(x, y, width, height)
        self.left_bound, self.right_bound = left_bound, right_bound

        # Physical properties
        self.thrust = 3.5
        self.damping = 0.465
        self.inertia = 0.5
        self.velocity = 0
        self.acceleration = 0
        self.mu_s, self.mu_k = 1, 2
        self.switch = True
        self.pid = PID(kp=0.075, ki=0.01, kd=0.1)
        
    def move(self):
        self.enforce_bounds()
        self.velocity += self.acceleration
        # self.velocity = max(-10, min(10, self.velocity))  # Limit the velocity to a range
        self.friction()
        self.x += self.velocity
        self.acceleration = 0 # reset acceleration calculation for next frame

    def accelerate(self, force): # Exert a force on the spaceship
        self.acceleration += force / self.inertia

    def thrust_right(self, thrust=None):
        if thrust is None:
            thrust = self.thrust
        self.accelerate(thrust)
    
    def thrust_left(self, thrust=None):
        if thrust is None:
            thrust = self.thrust
        self.accelerate(-thrust)
        
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

    def pid_control(self, target):
        self.accelerate(self.pid.output(target, self.center_x))
    
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

class Breadcrumb(pygame.Rect):
    def __init__(self, x, y, width, height, padding, top_padding):
        super().__init__(x - padding//2, y - padding//2 - top_padding, width + padding//2, height + padding//2 + top_padding)
        self.collisions = []
    
    def shift(self, dx):
        self.x += dx

    @property
    def center_x(self):
        return self.x + self.width//2
    
    @property
    def center_y(self):
        return self.y + self.height//2

class Path:
    def __init__(self, width, height, x, y_bot, y_top, num_breadcrumbs=10, padding=100, top_padding=20):
        self.x = x
        dy = abs(y_top - y_bot) / num_breadcrumbs
        self.breadcrumbs = [Breadcrumb(x, int(y_bot - i*dy), width, height, padding, top_padding) for i in range(num_breadcrumbs)]
        self.cemented = set()

    def reset_breadcrumbs(self):
        for breadcrumb in self.breadcrumbs:
            breadcrumb.x = self.x
            breadcrumb.collisions = []
    
    def check_collisions(self, asteroids):
        for asteroid in asteroids:
            for breadcrumb in self.breadcrumbs:
                if breadcrumb.colliderect(asteroid):
                    breadcrumb.collisions.append(asteroid)

    def avoid_collisions(self, asteroids, dx):
        self.cemented = set()
        for idx, breadcrumb in enumerate(self.breadcrumbs):
            if breadcrumb.collisions:
                self.cemented.add(idx)
                # adjust until no collision for current breadcrumb
                while breadcrumb.collisions: 
                    collisions = []
                    for asteroid in breadcrumb.collisions:
                        if breadcrumb.colliderect(asteroid):
                            breadcrumb.shift(dx)
                            collisions.append(asteroid)
                    breadcrumb.collisions = collisions
            # if current breadcrumb has no collisions, shift back to original position
            if not breadcrumb.collisions: 
                diff = breadcrumb.x - self.x
                if diff != 0:
                    breadcrumb.shift(-diff)
                    # undo shift if original position collides with an asteroid
                    for asteroid in asteroids:
                        if breadcrumb.colliderect(asteroid):
                            breadcrumb.shift(diff)
    
    def avoid_interpolation_collisions(self, asteroids, buffer, screen):
        for idx, breadcrumb in enumerate(self.breadcrumbs):
            if idx < buffer:
                continue
            prev_breadcrumb = self.breadcrumbs[idx-buffer]
            left_x = min(prev_breadcrumb.x, breadcrumb.x)
            right_x = max(prev_breadcrumb.x, breadcrumb.x)
            width = right_x - left_x
            y = (prev_breadcrumb.y + breadcrumb.y) // 2
            # height = abs(breadcrumb.center_y - prev_breadcrumb.center_y)
            interpolation = pygame.Rect(left_x, breadcrumb.center_y, width, breadcrumb.height)
            for asteroid in asteroids:
                if interpolation.colliderect(asteroid):
                    # pygame.draw.rect(screen, (255,255,255), interpolation)
                    breadcrumb.x = prev_breadcrumb.x
                    self.cemented.add(idx)
            # pygame.draw.rect(screen, (255,255,255), interpolation)
    
    # smooth without taking into account new positions
    def apply_smoothing(self, iterations):
        for _ in range(iterations):
            xs = [breadcrumb.x for breadcrumb in self.breadcrumbs]
            for idx, breadcrumb in enumerate(self.breadcrumbs):
                if idx in self.cemented:
                    continue
                if idx == 0:
                    if breadcrumb.x != self.x:
                        continue
                    xs[idx] = (breadcrumb.x + self.breadcrumbs[idx+1].x) // 2
                    continue
                if idx == len(self.breadcrumbs) - 1:
                    if breadcrumb.x != self.x:
                        continue
                    xs[idx] = (breadcrumb.x + self.breadcrumbs[idx-1].x) // 2
                    continue
                xs[idx] = (self.breadcrumbs[idx-1].x + breadcrumb.x + self.breadcrumbs[idx+1].x) // 3
            for idx, breadcrumb in enumerate(self.breadcrumbs):
                breadcrumb.x = xs[idx]

    # smooth taking into account new positions
    def apply_smoothing_extra(self, iterations, filter_size=1):
        for _ in range(iterations):
            xs = [breadcrumb.x for breadcrumb in self.breadcrumbs]
            for idx in range(len(self.breadcrumbs)):
                if idx < filter_size or idx >= len(self.breadcrumbs) - filter_size:
                    continue
                xs[idx] = sum([breadcrumb.x for breadcrumb in self.breadcrumbs[idx-filter_size:idx+filter_size+1]]) / (2*filter_size + 1)
            for idx, breadcrumb in enumerate(self.breadcrumbs):
                breadcrumb.x = xs[idx]
    
    # Adjust breadcrumbs depending on collisions with asteroids
    def adjust(self, new_x, asteroids, dx, screen):
        # self.x = new_x
    
        # reset all breadcrumbs
        self.reset_breadcrumbs()
        
        # check for collisions
        self.check_collisions(asteroids)
        
        # adjust based on collisions
        self.avoid_collisions(asteroids, dx)
        
        # adjust if path from previous breadcrumb to current breadcrumb collides with an asteroid
        self.avoid_interpolation_collisions(asteroids, 1, screen)
        # smooth out points
        self.apply_smoothing(5)
        self.apply_smoothing_extra(10)

        # # adjust if path from previous breadcrumb to current breadcrumb collides with an asteroid
        # self.avoid_interpolation_collisions(asteroids, buffer=2)
        
    def get_target(self, idx, screen):
        # pygame.draw.rect(screen, (255, 0, 0), self.breadcrumbs[idx])
        return self.breadcrumbs[idx].center_x
    
    def draw(self, screen):
        for breadcrumb in self.breadcrumbs:
            crumb = pygame.Rect(breadcrumb.center_x, breadcrumb.center_y, 2, 2)
            pygame.draw.rect(screen, WHITE, crumb)

class PID:
    def __init__(self, kp=0, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.last_error = 0
        self.integral = 0
        self.derivative = 0
    
    def output(self, target, current):
        error = target - current
        self.integral += error
        self.derivative = error - self.last_error
        self.last_error = error
        
        return self.kp * error + self.ki * self.integral + self.kd * self.derivative