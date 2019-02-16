import sys, pygame
import time
import math
import numpy as np


size = width, height = 800, 600
bgColor = 132, 206, 249

pygame.init()
screen = pygame.display.set_mode(size)

falcon = pygame.image.load("falcon9_2d_thrust.png")

# Get a rect of the original image, centered on the COM of the falcon within the image file
falconRect = falcon.get_rect(center=(74, 276))
falconPPM = 7.86  # Pixels per meter

# Falcon 9 position
x = -190
y = 300
dx = 90
dy = -100
theta = 1
dtheta = 0
grav = 9.8
t = time.time()
dryMass = 25000  # kg
propMass = 10000
momentInertia = 0.2 * (dryMass + propMass) * 47**2   # Or thereabouts

# Camera position (centers the specified world coordinate in the window)
camerax = 0
cameray = 0

# Camera zoom (when zoom == 1, pixels == meters)
camerazoom = 1

startTime = t

while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()

    ### PHYSICS


    # Time
    newT = time.time()
    dt = newT - t
    t = newT

    # Forces and torques

    c, s = np.cos(theta), np.sin(theta)
    rotMat = np.array(((c,-s), (s, c)))

    thrustAngleDeg = -1 * theta - dtheta + x * 0.001 + dx * 0.01
    thrustMag = 845000 - y * 10000 - dy * 30000 # N
    if thrustMag < 0: thrustMag = 0

    thrustAngle = thrustAngleDeg / 180.0 * math.pi
    thrust = np.array([thrustMag * math.sin(thrustAngle), thrustMag * math.cos(thrustAngle)])
    thrust = np.matmul(rotMat, thrust)
    thrustOffset = np.array([0, -12])
    thrustOffset = np.matmul(rotMat, thrustOffset)
    thrustTorque = np.cross(thrustOffset, thrust)


    # Gravity
    gravity = np.array([0, -grav * (dryMass + propMass)])

    
    dtheta += thrustTorque / momentInertia
    dx = dx + (thrust[0] + gravity[0]) / (dryMass + propMass) * dt
    dy = dy + (thrust[1] + gravity[1]) / (dryMass + propMass) * dt

    theta = theta + dtheta * dt
    x = x + dx * dt
    y = y + dy * dt

    print x, y, theta, dx, dy, dtheta


    # Scale the image to make pixels == meters * camerazoom
    scaledFalcon = pygame.transform.scale(falcon, (int(falconRect.w/falconPPM*camerazoom), int(falconRect.h/falconPPM*camerazoom)))

    # Rotate the scaled image
    rotatedFalcon = pygame.transform.rotate(scaledFalcon, theta * 180 / math.pi)

    # Get the rect of the rotated image with the same center as the original image
    transformedRect = rotatedFalcon.get_rect(center=falconRect.center)

    # Translate the rect to put the center at the origin
    transformedRect = transformedRect.move(-falconRect.x-falconRect.w/2, -falconRect.y-falconRect.h/2)

    # Translate the image to put the falcon 9 at the desired world position (x, y)
    transformedRect = transformedRect.move(x * camerazoom, -y * camerazoom)

    # Translate the image to put the camera's location in the center of the window
    transformedRect = transformedRect.move(-camerax * camerazoom + width / 2, -cameray * camerazoom + height / 2)

    
    screen.fill(bgColor)

    # Draw the image on to the buffer
    screen.blit(rotatedFalcon, transformedRect)

    # Flip buffers to display the image
    pygame.display.flip()
    pygame.time.wait(10)