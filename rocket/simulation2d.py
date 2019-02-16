import sys, pygame
import math
pygame.init()

size = width, height = 800, 600
bgColor = 132, 206, 249

screen = pygame.display.set_mode(size)

falcon = pygame.image.load("falcon9_2d_thrust.png")

# Get a rect of the original image, centered on the COM of the falcon within the image file
falconRect = falcon.get_rect(center=(74, 276))
falconPPM = 7.86  # Pixels per meter

# Falcon 9 position
x = 0
y = 0
theta = 0

# Camera position (centers the specified world coordinate in the window)
camerax = 0
cameray = 0

# Camera zoom (when zoom == 1, pixels == meters)
camerazoom = 2.5

while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()

    theta = theta + 0.01
    x = 100 * math.cos(theta)
    y = -100 * math.sin(theta)


    # Scale the image to make pixels == meters * camerazoom
    scaledFalcon = pygame.transform.scale(falcon, (int(falconRect.w/falconPPM*camerazoom), int(falconRect.h/falconPPM*camerazoom)))

    # Rotate the scaled image
    rotatedFalcon = pygame.transform.rotate(scaledFalcon, theta * 180 / math.pi)

    # Get the rect of the rotated image with the same center as the original image
    transformedRect = rotatedFalcon.get_rect(center=falconRect.center)

    # Translate the rect to put the center at the origin
    transformedRect = transformedRect.move(-falconRect.x-falconRect.w/2, -falconRect.y-falconRect.h/2)

    # Translate the image to put the falcon 9 at the desired world position (x, y)
    transformedRect = transformedRect.move(x * camerazoom, y * camerazoom)

    # Translate the image to put the camera's location in the center of the window
    transformedRect = transformedRect.move(-camerax * camerazoom + width / 2, -cameray * camerazoom + height / 2)

    
    screen.fill(bgColor)

    # Draw the image on to the buffer
    screen.blit(rotatedFalcon, transformedRect)

    # Flip buffers to display the image
    pygame.display.flip()
    pygame.time.wait(10)