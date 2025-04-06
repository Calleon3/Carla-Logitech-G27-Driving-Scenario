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


        #code to add road Obsticals 
        obst_bp = blueprint_library.filter('vehicle.audi.a2')
        #obst_bp = blueprint_library.filter('vehicle.ford.mustang') 
        
        clock = pygame.time.Clock()
        
        def spawnObst (a,b,c,d):
                
                chance = random.randint(1,10)
                if chance>=5:
                    offset = random.randint(0, d) 
                    x_offset = 0
                    y_offset = 0
                    if c==0 or c==180:
                        x_offset = offset
                    elif c==90 or c==270:
                        y_offset = offset
                
                    custom_transform  = carla.Transform(
                        location = carla.Location(x = a + x_offset, y = b + y_offset, z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=c, roll=0)
                        )
                    Obstacle = world.try_spawn_actor(obst_bp[0], custom_transform)
                    actor_list.append(Obstacle)
                
        '''
        #road 1        
        spawnObst(35,-2,0,280)
        spawnObst(35,2,0,280)
        #road 2
        spawnObst(392,44,90,190)
        spawnObst(396,44,90,190)
        #road 4
        spawnObst(338.5,156,270,90)
        spawnObst(335,156,270,90)
        #road 5
        spawnObst(132,130,180,130)
        spawnObst(132,133,180,130)
        #road 6 
        spawnObst(90,156,90,90)
        spawnObst(92,156,90,90)
        #road 8 
        spawnObst(-2,42,270,200)
        spawnObst(2,42,270,200)
        '''
                
           

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

            print("({:.2f}, {:.2f}, {:.2f})".format(x, y, z))
            

            
            
            

           
    finally:
        print('Destroying actors...')
        client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list])
        print('Done.')

if __name__ == '__main__':
    main()
