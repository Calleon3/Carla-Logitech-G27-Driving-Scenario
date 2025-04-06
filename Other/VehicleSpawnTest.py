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
        


        #code to add road Obsticals 
        frontvehicle_bp = blueprint_library.filter(' vehicle.dodge.charger_2020')
        backvehicle_bp = blueprint_library.filter(' vehicle.dodge.charger_2020') 
        
        clock = pygame.time.Clock()
        
        def spawnVehicle ():
                
               
                SP = random.randint(1,6)
                if SP == 1:
                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 140, y = 0 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=0, roll=0)
                        )
                    frontVehicle = world.try_spawn_actor(frontvehicle_bp[0], custom_transform)
                    actor_list.append(frontVehicle)

                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 130, y = 0 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=0, roll=0)
                        )
                    backVehicle = world.try_spawn_actor(backvehicle_bp[0], custom_transform)
                    actor_list.append(backVehicle)
                elif SP ==2:                    
                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 394, y = 140 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=90, roll=0)
                        )
                    frontVehicle = world.try_spawn_actor(frontvehicle_bp[0], custom_transform)
                    actor_list.append(frontVehicle)

                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 394, y = 130 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=90, roll=0)
                        )
                    backVehicle = world.try_spawn_actor(backvehicle_bp[0], custom_transform)
                    actor_list.append(backVehicle)
                elif SP ==3:                    
                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 337, y = 200 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=270, roll=0)
                        )
                    frontVehicle = world.try_spawn_actor(frontvehicle_bp[0], custom_transform)
                    actor_list.append(frontVehicle)

                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 337, y = 210 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=270, roll=0)
                        )
                    backVehicle = world.try_spawn_actor(backvehicle_bp[0], custom_transform)
                    actor_list.append(backVehicle)
                elif SP ==4:                    
                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 200, y = 132, z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=180, roll=0)
                        )
                    frontVehicle = world.try_spawn_actor(frontvehicle_bp[0], custom_transform)
                    actor_list.append(frontVehicle)

                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 210, y = 132, z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=180, roll=0)
                        )
                    backVehicle = world.try_spawn_actor(backvehicle_bp[0], custom_transform)
                    actor_list.append(backVehicle)
                elif SP ==5:                    
                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 91, y = 200 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=90, roll=0)
                        )
                    frontVehicle = world.try_spawn_actor(frontvehicle_bp[0], custom_transform)
                    actor_list.append(frontVehicle)

                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 91, y = 190 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=90, roll=0)
                        )
                    backVehicle = world.try_spawn_actor(backvehicle_bp[0], custom_transform)
                    actor_list.append(backVehicle)
                elif SP ==6:                    
                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 0, y = 140 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=270, roll=0)
                        )
                    frontVehicle = world.try_spawn_actor(frontvehicle_bp[0], custom_transform)
                    actor_list.append(frontVehicle)

                    custom_transform  = carla.Transform(
                        location = carla.Location(x = 0, y = 150 , z = 1),
                        rotation = carla.Rotation(pitch= 0, yaw=270, roll=0)
                        )
                    backVehicle = world.try_spawn_actor(backvehicle_bp[0], custom_transform)
                    actor_list.append(backVehicle)
                

                spec_transform = carla.Transform(
                location = carla.Location(x = custom_transform.location.x - 10, y = custom_transform.location.y, z = custom_transform.location.z + 5),
                rotation = custom_transform.rotation 
                )
                spectator.set_transform(spec_transform)
        spawnVehicle()
        
           

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
