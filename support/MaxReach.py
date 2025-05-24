###############################################################
# meArm test kinematics, no arm movements
# Urs Utzinger 2025
#
# Max x 200 Min -220
# Max y 217 Min    7
# Max z 120 Min  -79
###############################################################
import time
import pygame
import math
import kinematics

# Constants
INCREMENT = 0.05
INTERVAL = 0.1

# Globals

x = 0.
y = 0.
z = 0.
b = 0.
s = 0.
e = 0.
DEG2RAD = math.pi/180.
RAD2DEG = 1./DEG2RAD

pygame.init()
screen = pygame.display.set_mode((400,300))
pygame.display.set_caption("Keyboard input")
clock = pygame.time.Clock()
font=pygame.font.SysFont(None, 36)
font_small=pygame.font.SysFont(None, 16)

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))
    
        
def checkKey():
    global x,y,z,b,s,e

    current_time = time.time()
    keys = pygame.key.get_pressed()

    if   keys[pygame.K_UP]:
            s +=  INCREMENT

    elif keys[pygame.K_DOWN]: 
            s -=  INCREMENT
            
    elif keys[pygame.K_RIGHT]: 
            b +=  INCREMENT
            
    elif keys[pygame.K_LEFT]: 
            b -=  INCREMENT

    elif keys[pygame.K_a]: ## Moves meArm UP when "u" key is pressed
            e +=  INCREMENT

    elif keys[pygame.K_s]: ## Moves meArm DOWN when "j" key is pressed
            e -=  INCREMENT

    else:
        pass

    angleMinBase     =   -90.*DEG2RAD
    angleMaxBase     =  90.*DEG2RAD
    angleMinShoulder =     0.*DEG2RAD
    angleMaxShoulder = 150.*DEG2RAD
    angleMinElbow    =  -180.*DEG2RAD
    angleMaxElbow    =  30.*DEG2RAD

    b = clamp(b,angleMinBase, angleMaxBase)
    s = clamp(s,angleMinShoulder, angleMaxShoulder)
    e = clamp(e,angleMinElbow, angleMaxElbow)

    x, y, z = kinematics.unsolve(b, s, e)

def updateText():
    global x,y,z,b,s,e
    
    text0 = font.render(f"X: {x}", True, (255, 255, 255))
    text1 = font.render(f"y: {y}", True, (255, 255, 255))
    text2 = font.render(f"Z: {z}", True, (255, 255, 255))

    text3 = font.render(f"B: {b*RAD2DEG}", True, (255, 255, 255))
    text4 = font.render(f"S: {s*RAD2DEG}", True, (255, 255, 255))
    text5 = font.render(f"E: {e*RAD2DEG}", True, (255, 255, 255))
    
    help_1 = font_small.render("Base: Up/Down", True, (255, 255, 255))
    help_2 = font_small.render("Shoulder: Left/Right", True, (255, 255, 255))
    help_3 = font_small.render("Elbow: a/s", True, (255, 255, 255))

    # Blit text to screen
    screen.fill((0, 0, 0)) 
    screen.blit(text0, (20, 20))
    screen.blit(text1, (20, 60))
    screen.blit(text2, (20, 100))
    screen.blit(text3, (20, 140))
    screen.blit(text4, (20, 180))
    screen.blit(text5, (20, 220))
    screen.blit(help_1, (20, 240))
    screen.blit(help_2, (20, 260))
    screen.blit(help_3, (20, 280))
    pygame.display.flip()

###################################################
# Main Loop
###################################################



keys = pygame.key.get_pressed()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            quit()

        else:
            pass
    
    checkKey()
    updateText()
    
    clock.tick(30)

