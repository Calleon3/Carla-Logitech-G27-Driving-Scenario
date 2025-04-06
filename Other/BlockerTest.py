#!/usr/bin/env python

import glob
import os
import sys
import pygame
import numpy as np
import carla
import random
import math
import time 

def find_carla_egg():
    try:
        return glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0]
    except IndexError:
        raise RuntimeError("CARLA egg not found")

#def initialize_pygame():
    #pygame.init()
    #return pygame.display.set_mode((1920, 1080), pygame.HWSURFACE | pygame.DOUBLEBUF)


def main():
    
    
    actor_list = []
    
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)
        world = client.get_world()
        actors = world.get_actors()
        blueprint_library = world.get_blueprint_library()

        spectator = world.get_spectator()
        spec_transform = carla.Transform(
            #Top of screen
            #location = carla.Location(x = 200, y = 250, z = 400),
            #Secondary
            location = carla.Location(x = 90, y = 10, z = 50),
            rotation = carla.Rotation(pitch=-90, yaw=0, roll=0)
        )
        spectator.set_transform(spec_transform)


        #code to add road blockers 
        blockers_bp = blueprint_library.filter('vehicle.mitsubishi.fusorosa')
        #blockers_bp = blueprint_library.filter('vehicle.ford.mustang') 
        
        clock = pygame.time.Clock()

        def spawnblocker (a,b,c):
                custom_transform  = carla.Transform(
                    location = carla.Location(x = a, y = b, z = 1),
                    rotation = carla.Rotation(pitch= 0, yaw=c, roll=0)
                    )
                blocker = world.spawn_actor(blockers_bp[0], custom_transform)
                actor_list.append(blocker)
                
                
        spawnblocker(91,8,0)
        spawnblocker(157,8,0)
        spawnblocker(337,8,0)
        spawnblocker(91,122,0)
        spawnblocker(337,122,0)
        #spawnblocker(100,20,)
        spawnblocker(98,198,90)
        spawnblocker(328,198,90)
        spawnblocker(98,329,90)
        spawnblocker(328,329,90)        
            

        while True:
            world.tick()
            #pygame.display.flip()
            clock.tick(30)
            spectator_transform = spectator.get_transform()
            spectator_location = spectator_transform.location

            # Access individual coordinates
            x = spectator_location.x
            y = spectator_location.y
            z = spectator_location.z

            #print("({:.2f}, {:.2f}, {:.2f})".format(x, y, z))


            
            
            

           
    finally:
        print('Destroying actors...')
        client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list])
        print('Done.')

if __name__ == '__main__':
    main()
