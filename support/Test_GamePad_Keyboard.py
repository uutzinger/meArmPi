import pygame
import time

pygame.init()
pygame.joystick.init()

try:
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Controller detected: ", joystick.get_name())
except pygame.error:
    print("No joystick detected.")
    quit()

screen = pygame.display.set_mode((400,300))
pygame.display.set_caption("Keyboard input")
clock = pygame.time.Clock()
check_joy_time = time.time()
INTERVAL_USERINPUT = 0.03

while True:
    current_time = time.time()
    
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            quit()

        elif event.type == pygame.JOYBUTTONDOWN:
            print("Button pressed: ", event.button)
            
        elif event.type == pygame.JOYAXISMOTION:
            if (current_time - check_joy_time) > INTERVAL_USERINPUT:
                print("Axis moved: ", event.axis, event.value)
                check_joy_time = current_time
                current_time = time.time()

        elif event.type == pygame.KEYDOWN:
            print("Key pressed", event.key)
            
        elif event.type == pygame.JOYHATMOTION:
            print("Hat pressed:", event.value)

    clock.tick(100)