#Doesn't work
#Not using this line os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1' or steering initialise 



#!/usr/bin/env python
import glob
import os
import sys
import pygame
import numpy as np
import random
import math
import ctypes
import time
from logidrivepy import LogitechController


pygame.init()
screen = pygame.display.set_mode((800, 600))
joystick = pygame.joystick.Joystick(0)
joystick.init()

dll_path = "C:\Program Files\Logitech\Gaming Software\SDKs\LogitechSteeringWheel.dll"
# Initialize the controller
controller = LogitechController(dll_path)


def get_steering_input():
    pygame.event.pump()
    steering = joystick.get_axis(0)  # Axis 0 is typically the steering axis
    return steering

def main():
    try:
        clock = pygame.time.Clock()
        
        
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
            controller.logi_update()  
            controller.play_spring_force(0,0,50,50)
            controller.logi_update()    
            #Optional: Get steering input
            steering = get_steering_input()
            print(steering)
            
            clock.tick(30)
    
    finally:
        # Cleanup
        print("cleanup")
        controller.stop_constant_force(0)
        controller.stop_spring_force(0)
        controller.steering_shutdown()
        pygame.quit()
        sys.exit(0)

if __name__ == '__main__':
    main()