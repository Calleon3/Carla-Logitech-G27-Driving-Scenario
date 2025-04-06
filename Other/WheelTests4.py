#works implement to new haptic 
#Do in lab but add camera and stream to pygame and see if it works 


#!/usr/bin/env python

import glob
import os
import sys
import carla
import random
import time
import math
import numpy as np
from agents.navigation.controller import VehiclePIDController
from logidrivepy import LogitechController
os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
# Initialize Pygame
import pygame
pygame.init
screen = pygame.display.set_mode((1000, 800))
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

dll_path = "C:\Program Files\Logitech\Gaming Software\SDKs\LogitechSteeringWheel.dll"
# Initialize the controller
controller = LogitechController(dll_path)
controller.steering_initialize()

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

def get_steering_input():
    pygame.event.pump()
    steering = joystick.get_axis(0)  # Axis 0 is typically the steering axis
    return steering


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

def process_img(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array)
    surface = pygame.transform.flip(surface, True, False)  # Flip horizontally
    screen.blit(surface, (0,0))
    pygame.display.flip()

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

        spawn_points = world.get_map().get_spawn_points()
        random_spawn_point = random.choice(spawn_points)
        #waypoint = world.get_map().get_waypoint(carla.Location(x=6, y=3, z=0.5))
        #random_spawn_point = waypoint.transform
        frontvehicle = world.try_spawn_actor(frontvehicle_bp[0], random_spawn_point)
        actor_list.append(frontvehicle)

        target_transform = carla.Transform(
                location = carla.Location(x = random_spawn_point.location.x + 50*random_spawn_point.rotation.get_forward_vector().x, y = random_spawn_point.location.y +50 * random_spawn_point.rotation.get_forward_vector().y, z = random_spawn_point.location.z),
                rotation = random_spawn_point.rotation 
                )
        target_waypoint = world.get_map().get_waypoint(target_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving) 
        
        spec_transform = carla.Transform(
                location = carla.Location(x = random_spawn_point.location.x - 0*random_spawn_point.rotation.get_forward_vector().x, y = random_spawn_point.location.y - 0 * random_spawn_point.rotation.get_forward_vector().y, z = random_spawn_point.location.z + 5),
                rotation = random_spawn_point.rotation 
                )
        spectator.set_transform(spec_transform)

        camera_rotation = carla.Rotation(pitch=0, yaw=0, roll=90)
        camera_transform = carla.Transform(carla.Location(x=0.2,y=-0.4, z=1.25), camera_rotation)
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1080')
        camera_bp.set_attribute('image_size_y', '1920')
        camera_bp.set_attribute('fov', '115')
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=frontvehicle)
        actor_list.append(camera)
        
        camera.listen(process_img)

       

        # Apply the control to the vehicle
        #frontvehicle.apply_control(vehicle_control)

        while True:
            world.tick()
            #pygame.display.flip()

            controller.logi_update()  
            controller.play_spring_force(0,0,50,50)
            controller.logi_update()    
            steering = get_steering_input()
            print(steering)
           
            
            clock.tick(30)    
            
            

    finally:
        print('Destroying actors...')
        if client and world: #Check that client and world are valid
            client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list if actor])
            #added if actor to ensure only valid actors are destroyed
        else:
            print("Client or world not initialized properly.")
        print('Done.')
        controller.stop_constant_force(0)
        controller.stop_spring_force(0)
        controller.steering_shutdown()
        pygame.quit()
        sys.exit(0)

if __name__ == '__main__':
    main()
