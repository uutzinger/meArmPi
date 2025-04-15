import pygame

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

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            quit()

        if event.type == pygame.JOYBUTTONDOWN:
            print("Button pressed: ", event.button)
            
        if event.type == pygame.JOYAXISMOTION:
            print("Axis moved: ", event.axis, event.value)

        if event.type == pygame.KEYDOWN:
            print("Key pressed", event.key)
    clock.tick(60)