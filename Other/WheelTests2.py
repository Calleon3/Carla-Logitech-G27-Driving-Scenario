#works
#uses both os.environ... and steering_initiialsie 


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
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'

pygame.init()
from pygame.locals import K_SPACE
screen = pygame.display.set_mode((800, 600))
joystick = pygame.joystick.Joystick(0)
joystick.init()



'''
range_value = ctypes.c_long()
range_ptr = ctypes.pointer(range_value)
print(controller.set_operating_range(0, 100))
print(controller.get_operating_range(0, range_ptr))
print(range_value.value)
'''
def get_steering_input():
    pygame.event.pump()
    steering = joystick.get_axis(0)  # Axis 0 is typically the steering axis
    return steering

def main():
    try:
        clock = pygame.time.Clock()
        dll_path = "C:\Program Files\Logitech\Gaming Software\SDKs\LogitechSteeringWheel.dll"
        # Initialize the controller
        controller = LogitechController(dll_path)
        controller.steering_initialize()
        
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
            controller.logi_update()  
            controller.play_spring_force(0,0,20,20)
            controller.logi_update()    

            #Optional: Get steering input
            steering = get_steering_input()
            #print(steering)
            
            clock.tick(30)
    
    finally:
        # Cleanup
        print("cleanup")
        controller.stop_constant_force(0)
        controller.stop_spring_force(0)
        controller.steering_shutdown()
        pygame.quit()
        sys.exit(0)

while True:    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                if __name__ == '__main__':
                    main()