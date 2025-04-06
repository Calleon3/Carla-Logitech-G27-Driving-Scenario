#!/usr/bin/env python

import glob
import os
import sys
import pygame
import carla
import random
import time

pygame.init()  # Initialize Pygame

from pygame.locals import K_w, K_a, K_s, K_d

def find_carla_egg():
    try:
        return glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0]
    except IndexError:
        raise RuntimeError("CARLA egg not found")

def initialize_pygame():
    pygame.init()
    return pygame.display.set_mode((800, 600))  # Create a display window (adjust size as needed)

def main():
    actor_list = []
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()
        clock = pygame.time.Clock()
        spectator = world.get_spectator()

        blockers_bp = blueprint_library.filter('vehicle.ford.mustang')
        custom_transform = carla.Transform(
            location=carla.Location(x=101, y=0, z=1),
            rotation=carla.Rotation(pitch=0, yaw=0, roll=0)
        )

        backvehicle = world.spawn_actor(blockers_bp[0], custom_transform)
        actor_list.append(backvehicle)

        screen = initialize_pygame() # Initialize pygame
        T=0.0
        B=0.0
        S=0.0
        while True:
            world.tick()
            clock.tick(30)

            for event in pygame.event.get(): # Process events
                if event.type == pygame.QUIT: # Handle window close event
                    return # Exit main loop

                if event.type == pygame.KEYDOWN:
                    if event.key == K_w:
                        T = 0.5
                    if event.key == K_s:
                        T = -0.5
                    if event.key == K_a:
                        S = -0.3
                    if event.key == K_d:
                        S = 0.3

            control = carla.VehicleControl(throttle=max(min(T, 1.0), -1.0), brake=B, steer=max(-1.0, min(1.0, S)))
            backvehicle.apply_control(control)


            vehicle_transform = backvehicle.get_transform()
            forward_vector = vehicle_transform.get_forward_vector()
        
            # Calculate camera position behind the vehicle
            camera_location = vehicle_transform.location - 10 * forward_vector
            camera_location.z += 5  # Raise the camera slightly
        
            # Calculate camera rotation to look at the vehicle
            camera_rotation = vehicle_transform.rotation
            camera_rotation.pitch -= 15  # Tilt the camera down slightly
        
            # Set spectator's transform
            camera_transform = carla.Transform(camera_location, camera_rotation)
            spectator.set_transform(camera_transform)
        
            # Add a small delay to avoid overwhelming the simulator
            world.wait_for_tick()

    finally:
        print('Destroying actors...')
        client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list])
        pygame.quit() # important to call quit()
        print('Done.')

if __name__ == '__main__':
    main()
            

          
           


            
            
            

  