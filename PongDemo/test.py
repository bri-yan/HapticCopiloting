# P0ng by Aiden C
# Comp Sci 2023

import math
import time
import random
import pygame

#######################################################################################
#                                       Config                                        #
#######################################################################################

debug = False  # True = Debug On, False = Debug Off

# Client Settings
clientfps = 144  # Game FPS
fluidfps = True  # Change FPS to avoid collision failures

# Board Settings
board = (0, 0, 0)  # Board Color
arenasize = 15  # Arena Borders Offset From Display Edges (px)
goalsize = 200  # Goal Size (px)
respawntime = 3  # Respawn Time (blinks)

# Ball Settings
bx = 400  # Ball Starting X Pos
by = 300  # Ball Starting Y Pos
ballradius = 10  # Ball Starting Radius (px)
bdeg = 0  # Ball Starting Angle (degrees)
clientbvel = 300  # Ball Starting Velocity (px/s)
bounceunc = 5  # Uncertainty of Ball Bounce (deg) (0 = Perfect Physics) (180 = 100% uncertainty)
baccel = 10  # Ball Acceleration on Paddle Hit (px/s)
vellimit = 1500  # Maximum Ball Velocity

# Paddle Settings
p1y = 300  # Paddle 1 Starting Pos
p2y = 300  # Paddle 2 Starting Pos
pwidth = 15  # Paddles Width (px)
pheight = 100  # Paddles Height (px)
poffset = 100  # Paddles Offset From Edge (px)
pvel = 250  # Paddles Movement Velocity (px/s)
pcurve = 2  # Curve on Paddle Edges (px)
limits = True  # Enable Board Limits For Paddles

# 2-D Paddle Movement
p2Dm = False

#######################################################################################
#                                Initial Definitions                                  #
#######################################################################################

# Init Variables
pygame.init()
black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
green = (0, 255, 0)
yellow = (255, 255, 0)
debugfont = pygame.font.Font('freesansbold.ttf', 32)
scorefont = pygame.font.Font('freesansbold.ttf', 100)
pausedfont = pygame.font.Font('freesansbold.ttf', 50)
size = (800, 600)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("P1ng P0ng")
clock = pygame.time.Clock()
running = True

# Defining Starting Variables
obx = bx
oby = by
bvel = clientbvel
recangle = 0
impangle = 0
frames = 0
fpsrec = 0
left = 0
right = 0
bounce = False
blinks = 0
btransparency = 255
restart = False
fps = clientfps
pause = 0
p1x = (poffset - (pwidth / 2))
p2x = 800 - (poffset + (pwidth / 2))
oldpos = [0,0]
newcol = True


#######################################################################################
#                                    Functions                                        #
#######################################################################################

def circcol(rectangles, x, y, radius, density=1):  # Physics Engine For the Ball
    global paddle_left
    global paddle_right
    global border_top
    global border_bottom
    global debug
    global bvel
    hitcoord = []
    accel = False

    # Finding Collision Points
    for i in range(0, round((360 / density))):
        point = [(math.cos(math.radians(i * density)) * radius) + x, (math.sin(math.radians(i * density)) * radius) + y]
        for j in range(0, len(rectangles)):
            if rectangles[j].collidepoint(point):
                if j == 0 or j == 1:
                    accel = True
                hitcoord.append(point)
        if debug:
            pygame.draw.circle(screen, red, point, 1)

    # Averaging Return Value
    if hitcoord != []:
        sumx = 0
        sumy = 0
        for i in hitcoord:
            sumx += i[0]
            sumy += i[1]
        hitcoord = [(sumx / len(hitcoord)), (sumy / len(hitcoord))]
    if accel and bvel < vellimit:
        if newcol:
            bvel = bvel + baccel
    return hitcoord


#######################################################################################
#                                     Main Loop                                       #
#######################################################################################

while running:

    # Init Display and Variables
    pygame.display.flip()
    clock.tick(fps)
    screen.fill(board)
    keys = pygame.key.get_pressed()
    rectangles = []

    # Window Decoration
    for i in range(0, int((600 - (arenasize * 2)) / 20)):
        if blinks == 0:
            pygame.draw.line(screen, white, [400, (i * 20) + arenasize + 5], [400, ((i * 20) + 10) + arenasize + 5], int((800 - (arenasize * 2)) / 150))

    scoreleft = scorefont.render(str(left), True, white)
    scoreleftRect = scoreleft.get_rect()
    scoreleftRect.center = (200, 100)
    screen.blit(scoreleft, scoreleftRect)

    scoreright = scorefont.render(str(right), True, white)
    scorerightRect = scoreright.get_rect()
    scorerightRect.center = (600, 100)
    screen.blit(scoreright, scorerightRect)

    # Main Objects
    if goalsize == -1:
        goalsize = 10000
    rectangles.append(pygame.draw.rect(screen, white, (p1x, p1y - (pheight / 2), pwidth, pheight), 0, pcurve))  # Left Paddle
    rectangles.append(pygame.draw.rect(screen, white, (p2x - pwidth, p2y - (pheight / 2), pwidth, pheight), 0, pcurve))  # Right Paddle
    rectangles.append(pygame.draw.rect(screen, white, (arenasize, arenasize - 5, 800 - (arenasize * 2), 5)))  # Top Border
    rectangles.append(pygame.draw.rect(screen, white, (arenasize, 600 - arenasize, 800 - (arenasize * 2), 5)))  # Bottom Border
    rectangles.append(pygame.draw.rect(screen, white, (arenasize - 5, arenasize - 5, 5, (300 - (goalsize / 2) - arenasize) + 5)))  # Left Top Border
    rectangles.append(pygame.draw.rect(screen, white, (arenasize - 5, 300 + (goalsize / 2), 5, (600 - arenasize) - (300 + (goalsize / 2)) + 5)))  # Left Bottom Border
    rectangles.append(pygame.draw.rect(screen, white, (800 - arenasize, arenasize - 5, 5, (300 - (goalsize / 2) - arenasize) + 5)))  # Right Top Border
    rectangles.append(pygame.draw.rect(screen, white, (800 - arenasize, 300 + (goalsize / 2), 5, (600 - arenasize) - (300 + (goalsize / 2)) + 5)))  # Right Bottom Border
    rectangles.append(pygame.draw.rect(screen, white, (-100, 300 - (goalsize / 2) - 5, arenasize + 100, 5)))  # Right Top Goal
    rectangles.append(pygame.draw.rect(screen, white, (-100, 300 + (goalsize / 2), arenasize + 100, 5)))  # Right Bottom Goal
    rectangles.append(pygame.draw.rect(screen, white, (800 - arenasize, 300 - (goalsize / 2) - 5, arenasize + 100, 5)))  # Left Top Goal
    rectangles.append(pygame.draw.rect(screen, white, (800 - arenasize, 300 + (goalsize / 2), arenasize + 100, 5)))  # Left Bottom Goal

    ball = pygame.draw.circle(screen, (btransparency, btransparency, btransparency), [bx, by], ballradius)
    screen.blit(screen, (0, 0))

    # Handling Collision
    colpoint = circcol(rectangles, bx, by, ballradius)

    if colpoint == []:
        bounce = False
        newcol = True

    if (math.sqrt(((bx-oldpos[0])**2) + (by-oldpos[1])**2)) > 10 and bounce:
        bdeg = math.degrees(math.atan2(300 - by, 400 - bx))
        if debug:
            print("whelp, I messed up")

    if (colpoint != []) and (not bounce):
        print("Colpoint", colpoint)
        print("Subtracting", colpoint[1], "by", round(by), "for y of collision")
        print("Subtracting", colpoint[0], "by", round(bx), "for x of collision")
        impangle = round((math.degrees(math.atan2(colpoint[1] - by, colpoint[0] - bx))) % 360)
        print("Original angle:", bdeg)
        print("Collided at angle", impangle)
        recangle = bdeg
        oldpos = [bx,by]
        if bounceunc > 0:
            bdeg = ((((impangle - ((bdeg - impangle) % 360)) + 180) % 360) + random.randrange(int(math.floor(0 - (bounceunc / 2))), int(math.ceil(0 + (bounceunc / 2))))) % 360
        else:
            bdeg = (((impangle - ((bdeg - impangle) % 360)) + 180) % 360)
        print("Collided at angle", impangle, "and changed resulting angle to", bdeg)
        print("-------------------------------------------------------")
        print()
        bounce = True
        newcol = False

    if debug:
        pygame.draw.line(screen, red, [400, 300], [(400 + (100 * math.cos(math.radians(recangle)))), (300 + (100 * math.sin(math.radians(recangle))))], 5)
        pygame.draw.line(screen, yellow, [400, 300], [(400 + (80 * math.cos(math.radians(impangle)))), (300 + (80 * math.sin(math.radians(impangle))))], 5)
        pygame.draw.line(screen, green, [400, 300], [(400 + (60 * math.cos(math.radians(bdeg)))), (300 + (60 * math.sin(math.radians(bdeg))))], 5)

        text = debugfont.render("P1y: " + str(p1y) + "  Ball Vel: " + str(bvel) + "px/s  FPS: " + str(fpsrec) + "  P2y: " + str(p2y), True, pygame.Color(255, 255, 255), black)
        textRect = text.get_rect()
        textRect.center = (400, 40)
        screen.blit(text, textRect)

    # Moving the Ball
    if pause == 0:
        bx += round(math.cos(math.radians(bdeg)), 3) * round(bvel * (1 / fps), 3)
        by += round(math.sin(math.radians(bdeg)), 3) * round(bvel * (1 / fps), 3)

    if bx < 0 - ballradius:
        if (by > 300 - (goalsize / 2)) and (by < 300 + (goalsize / 2)):
            right += 1
        bx = 400
        by = 300
        bdeg = 0
        bvel = 0
        blinks = 1
        btransparency = 255
        fps = clientfps
        restart = True
    if bx > 800 + ballradius:
        if (by > 300 - (goalsize / 2)) and (by < 300 + (goalsize / 2)):
            left += 1
        bx = 400
        by = 300
        bdeg = 180
        bvel = 0
        blinks = 1
        btransparency = 255
        fps = clientfps
        restart = True
    if (blinks > 0) and (blinks < (respawntime * 2) + 1):
        transchange = (255 / fps) * 4
        if blinks % 2 == 1:
            btransparency -= transchange
            if btransparency - transchange <= 0:
                blinks += 1
        else:
            btransparency += transchange
            if btransparency + transchange >= 255:
                blinks += 1
    elif restart:
        bvel = clientbvel
        blinks = 0
        btransparency = 255
        restart = False

    if limits:
        arenalimits = arenasize
        centerlineleft = 380
        centerlineright = 420
    else:
        arenalimits = -50
        centerlineleft = 850
        centerlineright = -50

    if keys[pygame.K_w] and (p1y - (pheight / 2) > arenalimits + 5) and pause == 0:
        p1y -= round((pvel * (1 / fps)), 3)
    elif keys[pygame.K_s] and (p1y + (pheight / 2) < (600 - arenalimits) - 5) and pause == 0:
        p1y += round(pvel * (1 / fps), 3)

    if keys[pygame.K_UP] and (p2y - (pheight / 2) > arenalimits + 5) and pause == 0:
        p2y -= round(pvel * (1 / fps), 3)
    elif keys[pygame.K_DOWN] and (p2y + (pheight / 2) < (600 - arenalimits) - 5) and pause == 0:
        p2y += round(pvel * (1 / fps), 3)

    if p2Dm:
        if keys[pygame.K_a] and (p1x > (arenalimits + 5)) and pause == 0:
            p1x -= round((pvel * (1 / fps)), 3)
        elif keys[pygame.K_d] and (p1x + pwidth < centerlineleft) and pause == 0:
            p1x += round(pvel * (1 / fps), 3)

        if keys[pygame.K_LEFT] and (p2x - pwidth > centerlineright) and pause == 0:
            p2x -= round(pvel * (1 / fps), 3)
        elif keys[pygame.K_RIGHT] and (p2x < (800 - arenalimits) - 5) and pause == 0:
            p2x += round(pvel * (1 / fps), 3)

    p1y = round(p1y, 3)
    p2y = round(p2y, 3)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                if pause == 0:
                    pause = 1
                elif pause == 1:
                    pause = 2

    if fluidfps:
        if fps <= 60:
            if bvel > 800:
                fps = 120
        if fps <= 120:
            if bvel > 1600:
                fps = 180
        if (bvel > 1600) and (fpsrec > fps - 100) and (bvel / 10 > clientfps):
            fps = bvel / 10

    if pause == 1:
        text = pausedfont.render("Paused", True, pygame.Color(255, 255, 255), board)
        textRect = text.get_rect()
        textRect.center = (400, 300)
        screen.blit(text, textRect)
        white = (150, 150, 150)
        btransparency = 150
    elif (pause == 2):
        white = (255, 255, 255)
        btransparency = 255
        pause = 0

    t = time.localtime()
    seconds = time.strftime("%S", t)
    if (int(seconds) % 2) == 0:
        if frames > 1:
            fpsrec = frames
        frames = 0
    frames += 1

pygame.quit()