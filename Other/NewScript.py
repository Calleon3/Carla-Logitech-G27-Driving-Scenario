#!/usr/bin/env python

import glob
import os
import sys
import carla
import random
import time
import math
import numpy as np
from agents.navigation.behavior_agent import BehaviorAgent

# Initialize Pygame
import pygame
pygame.init
from pygame.locals import K_w, K_a, K_s, K_d

#Custom Functions 
def find_carla_egg():
    try:
        return glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0]
    except IndexError:
        raise RuntimeError("CARLA egg not found")

def wait_for_seconds(world, seconds):
    start_time = time.time()
    while time.time() - start_time < seconds:
        world.tick()


def setup_Map (client, world, actors, blueprint_library):
    #world.unload_map_layer(carla.MapLayer.Buildings)
    #world.unload_map_layer(carla.MapLayer.Foliage)
    #world.unload_map_layer(carla.MapLayer.Walls)

    #choose weather settings for test
    '''
    weather = carla.WeatherParameters(
        cloudiness=0.0,
        precipitation=0.0,
        sun_altitude_angle=90.0
    )
    world.set_weather(weather)
    '''

    vehicles = actors.filter('vehicle.*')
    client.apply_batch([carla.command.DestroyActor(actor) for actor in vehicles if actor])

    #sets all traffic lights to green for X seconds 
    for actor in actors:
        if isinstance(actor, carla.TrafficLight):
            actor.set_state(carla.TrafficLightState.Green)
            actor.set_green_time(1000.0)
    return 0

def main():
    #defining variables 
    actor_list = []
   
    
    try:
        #Joining server as client 
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        clock = pygame.time.Clock()
        #screen = pygame.display.set_mode((800, 600))

        #defining actor variables 
        world = client.get_world()
        actors = world.get_actors()
        blueprint_library = world.get_blueprint_library()
        spectator = world.get_spectator()    
        setup_Map (client, world, actors, blueprint_library)
            
        #Setting Blueprints 
        
        frontvehicle_bp = blueprint_library.filter('vehicle.dodge.charger_2020')  
        backvehicle_bp = blueprint_library.filter('vehicle.dodge.charger_2020')  
        altbackvehicle_bp = blueprint_library.filter('vehicle.mercedes.sprinter')  
        obstacle_bp = blueprint_library.filter('vehicle.audi.a2')

       

        


        while True:
            world.tick()
            #pygame.display.flip()
            clock.tick(30)    
            
            

    finally:
        print('Destroying actors...')
        if client and world: #Check that client and world are valid
            client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list if actor])
            #added if actor to ensure only valid actors are destroyed
        else:
            print("Client or world not initialized properly.")
        print('Done.')

if __name__ == '__main__':
    main()
