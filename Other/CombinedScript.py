#!/usr/bin/env python

import glob
import os
import sys
import carla
import random
import time
import math
import numpy as np

# Initialize Pygame and joystick
import pygame
pygame.init
screen = pygame.display.set_mode((800, 600)) # Initialize pygame
from pygame.locals import K_w, K_a, K_s, K_d
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

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

def SetSpectator(spectator,a,b,c,d,e,f):
    spec_transform = carla.Transform(           
            location = carla.Location(x = a, y = b, z = c),
            rotation = carla.Rotation(pitch=d, yaw=e, roll=f)
        )
    spectator.set_transform(spec_transform)

def spawnblocker (world,blockers_bp,actor_list,a,b,c):
                try: # Add a try-except block to handle potential spawning errors
                    custom_transform = carla.Transform(
                        location=carla.Location(x=a, y=b, z=1),
                        rotation=carla.Rotation(pitch=0, yaw=c, roll=0)
                    )
                    blocker = world.try_spawn_actor(blockers_bp[0], custom_transform)
                    if blocker: # Only add to actor_list if spawning was successful
                        actor_list.append(blocker)
                        return blocker # Return the spawned actor
                    else:
                        print(f"Warning: Failed to spawn blocker")
                        return None
                except Exception as e:
                    print(f"Error spawning blocker")
                    return None

def pd_controller(distance, relative_velocity, desired_distance, kp, kd):
            error = distance - desired_distance
            control = kp * error + kd * relative_velocity
            return max(min(control, 1.0), -1.0)  # Clamp between -1 and 1

def get_steering_input():
    pygame.event.pump()
    steering = joystick.get_axis(0)  # Axis 0 is typically the steering axis
    return steering

def calculate_steering(followerVehicle, lead_location):
            fwd = followerVehicle.get_transform().get_forward_vector()
            target_vector = lead_location - followerVehicle.get_transform().location
            dot = fwd.x * target_vector.x + fwd.y * target_vector.y
            cross = fwd.x * target_vector.y - fwd.y * target_vector.x
            steering = math.atan2(cross, dot) / math.pi
            return max(-1.0, min(1.0, steering))

def setup_Map (world, actors, blueprint_library):
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

    #sets all traffic lights to green for X seconds 
    for actor in actors:
        if isinstance(actor, carla.TrafficLight):
            actor.set_state(carla.TrafficLightState.Green)
            actor.set_green_time(1000.0)
    for TrafficLight in actors:
        TrafficLight.destroy()
        
    return 0

def main():
    #defining variables 
    actor_list = []
    kp = 100
    kd = 10
    desired_distance = 10
    x = 0
    y = 0
    manual = 1.0
    
    try:
        #Joining server as client 
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)
        clock = pygame.time.Clock()
        #screen = pygame.display.set_mode((800, 600))

        #defining actor variables 
        world = client.get_world()
        actors = world.get_actors()
        vehicles = actors.filter('vehicle.*')
        client.apply_batch([carla.command.DestroyActor(actor) for actor in vehicles if actor])
        blueprint_library = world.get_blueprint_library()
        spectator = world.get_spectator()    
        setup_Map (world, actor_list, blueprint_library)
            
        #Setting Blueprints 
        blockers_bp = blueprint_library.filter('vehicle.mitsubishi.fusorosa')
        frontvehicle_bp = blueprint_library.filter('vehicle.ford.mustang')  
        backvehicle_bp = blueprint_library.filter('vehicle.ford.mustang')  

        #Spawning Blockers
        
        spawnblocker(world,blockers_bp,actor_list,91,8,0)
        spawnblocker(world,blockers_bp,actor_list,157,8,0)
        spawnblocker(world,blockers_bp,actor_list,337,8,0)
        spawnblocker(world,blockers_bp,actor_list,91,122,0)
        spawnblocker(world,blockers_bp,actor_list,337,122,0)
        spawnblocker(world,blockers_bp,actor_list,98,198,90)
        spawnblocker(world,blockers_bp,actor_list,328,198,90)
        spawnblocker(world,blockers_bp,actor_list,98,329,90)
        spawnblocker(world,blockers_bp,actor_list,328,329,90)
        

        wait_for_seconds(world, 5)

        #Spawning Test Vehicles 
        transform = world.get_map().get_spawn_points()[10]
        frontvehicle = world.try_spawn_actor(frontvehicle_bp[0], transform)
        if frontvehicle:
            actor_list.append(frontvehicle)
        else:
            print("Failed to spawn front vehicle.")
        
        if transform.rotation.yaw == 0:
            x =-10
        elif transform.rotation.yaw == 90:
            y=-10
        elif transform.rotation.yaw == 180:
            x=10
        elif transform.rotation.yaw == 270:
            y=10
        back_transform = carla.Transform(
            location = carla.Location(x = transform.location.x + x, y = transform.location.y + y, z = transform.location.z),
            rotation = transform.rotation
        )
        backvehicle = world.try_spawn_actor(backvehicle_bp[0], back_transform)
        if backvehicle:
            actor_list.append(backvehicle)
        else:
            print("Failed to spawn back vehicle.")

        if frontvehicle:
            frontvehicle.set_autopilot(True)


        while True:
            world.tick()
            #pygame.display.flip()
            clock.tick(30)    
            
            if backvehicle and frontvehicle:
                #updating vehicle variables 
                back_transform = backvehicle.get_transform()
                back_vector = backvehicle.get_transform().get_forward_vector()   
                backLocation = backvehicle.get_location()
                backVelocity = backvehicle.get_velocity()

                front_transform = frontvehicle.get_transform()
                front_vector = frontvehicle.get_transform().get_forward_vector()   
                frontLocation = frontvehicle.get_location()
                frontVelocity = frontvehicle.get_velocity()
            
                eDistance = backLocation.distance(frontLocation)
                eVelocity = (frontVelocity - backVelocity).length()
            
                #Manual controller
                for event in pygame.event.get(): # Process events
                    if event.type == pygame.QUIT: # Handle window close event
                        return # Exit main loop

                    if event.type == pygame.KEYDOWN:
                        if event.key == K_w:
                            print("w")
                        if event.key == K_s:
                            print("s")
                        if event.key == K_a:
                            print("a")
                        if event.key == K_d:
                            print("d")

                #PD controller 
                throttle = pd_controller(eDistance, eVelocity, desired_distance, kp, kd)            
                steering = calculate_steering(backvehicle, frontLocation)
                steering = get_steering_input()
                #Combining control
                control = carla.VehicleControl(throttle=max(0, throttle), brake=abs(min(0, throttle)), steer=steering)
                backvehicle.apply_control(control)

                #Updating Spectator 
                # Calculate camera position behind the vehicle
                camera_location = front_transform.location - 10 * front_vector
                camera_location.z += 5  # Raise the camera slightly
        
                # Calculate camera rotation to look at the vehicle
                camera_rotation = front_transform.rotation
                camera_rotation.pitch -= 15  # Tilt the camera down slightly
        
                # Set spectator's transform
                camera_transform = carla.Transform(camera_location, camera_rotation)
                spectator.set_transform(camera_transform)

            world.wait_for_tick()

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        print('Destroying actors...')
        if client and world: #Check that client and world are valid
            client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list if actor])
            #added if actor to ensure only valid actors are destroyed
            pygame.quit() # important to call quit()
        else:
            print("Client or world not initialized properly.")
        print('Done.')

if __name__ == '__main__':
    main()
