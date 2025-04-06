#!/usr/bin/env python

import glob
import os
import sys
import pygame
import numpy as np
import carla
import random
import math
from logidrivepy import LogitechController
controller = LogitechController()
controller.steering_initialize()

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

def setup_camera(world, vehicle):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '400')
    camera_bp.set_attribute('image_size_y', '400')
    camera_bp.set_attribute('fov', '110')
    
    camera_transform = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    
    calibration = np.identity(3)
    calibration[0, 2] = 1920 / 2.0
    calibration[1, 2] = 1080 / 2.0
    calibration[0, 0] = calibration[1, 1] = 1920 / (2.0 * np.tan(110 * np.pi / 360.0))
    camera.calibration = calibration
    
    return camera

def setup_Map (world, actors, blueprint_library):
    #world.unload_map_layer(carla.MapLayer.Buildings)
    #world.unload_map_layer(carla.MapLayer.Foliage)
    #world.unload_map_layer(carla.MapLayer.Walls)

    #choose weather settings for test
    weather = carla.WeatherParameters(
        cloudiness=0.0,
        precipitation=0.0,
        sun_altitude_angle=90.0
    )
    world.set_weather(weather)

    #sets all traffic lights to green for X seconds 
    for actor in actors:
        if isinstance(actor, carla.TrafficLight):
            actor.set_state(carla.TrafficLightState.Green)
            actor.set_green_time(1000.0)


    return 0


#def process_image(image, display):
    #array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]
    #surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    #display.blit(surface, (0, 0))

def main():
    
    
    actor_list = []
    
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)
        world = client.get_world()
        actors = world.get_actors()
        blueprint_library = world.get_blueprint_library()

        #setups the map for tests
        setup_Map(world, actors, blueprint_library)

        #code to add road blockers 
        blockers_bp = blueprint_library.filter('vehicle.mitsubishi.fusorosa')
        
        custom_transform = world.get_map().get_spawn_points()[8]
        #carla.Transform(carla.Location(x=10, y=20, z=0), carla.Rotation(pitch=0, yaw=90, roll=0))
        blocker1 = world.spawn_actor(blockers_bp[0], custom_transform)
        actor_list.append(blocker1)

        #display = initialize_pygame()
        
        #choosing the vehicles to spawn
        frontvehicle_bp = blueprint_library.filter('vehicle.ford.mustang')  

        #selecting a random spawnpoint for the vehicle 
        transform = world.get_map().get_spawn_points()[10]
        x = 0
        y = 0
        if transform.rotation.yaw == 0:
            x =-10
        elif transform.rotation.yaw == 90:
            y=-10
        elif transform.rotation.yaw == 180:
            x=10
        elif transform.rotation.yaw == 270:
            y=10
        #setting transform for back vehicle to spawn behind 
        back_transform = carla.Transform(
            location = carla.Location(x = transform.location.x + x, y = transform.location.y + y, z = transform.location.z),
            rotation = transform.rotation
        )
        #sets spectator camera to where vehicle will spawn
        spectator = world.get_spectator()
        spec_transform = carla.Transform(
            location = carla.Location(x = transform.location.x - 10, y = transform.location.y, z = transform.location.z + 5),
            rotation = transform.rotation 
        )
        spectator.set_transform(spec_transform)
        
        #spawns vehicles
        frontvehicle = world.spawn_actor(frontvehicle_bp[0], transform)
        actor_list.append(frontvehicle)
        backvehicle = world.try_spawn_actor(frontvehicle_bp[0], back_transform)
        if backvehicle is None:
            back_transform = carla.Transform(
            location = carla.Location(x = transform.location.x, y = transform.location.y-10, z = transform.location.z),
            rotation = transform.rotation
            )
            backvehicle = world.try_spawn_actor(frontvehicle_bp[0], back_transform)
        
        actor_list.append(backvehicle)
        
        camera = setup_camera(world, frontvehicle)
        actor_list.append(camera)

        #camera.listen(lambda image: process_image(image, display))
        
        frontvehicle.set_autopilot(True)

        #PD controller for rear vehicle
        kp = 100
        kd = 10
        desired_distance = 10

        def pd_controller(distance, relative_velocity, desired_distance, kp, kd):
            error = distance - desired_distance
            control = kp * error + kd * relative_velocity
            return max(min(control, 1.0), -1.0)  # Clamp between -1 and 1

        def calculate_steering(followerVehicle, lead_location):
            fwd = followerVehicle.get_transform().get_forward_vector()
            target_vector = lead_location - followerVehicle.get_transform().location
            dot = fwd.x * target_vector.x + fwd.y * target_vector.y
            cross = fwd.x * target_vector.y - fwd.y * target_vector.x
            steering = math.atan2(cross, dot) / math.pi
            return max(-1.0, min(1.0, steering))
            
        
        
        clock = pygame.time.Clock()

        while True:
            world.tick()
            #pygame.display.flip()
            clock.tick(30)
            controller.logi_update()
            controller.LogiPlayConstantForce(50, 0)

            back_vector = backvehicle.get_transform().get_forward_vector()                      
            frontLocation = frontvehicle.get_location()
            backLocation = backvehicle.get_location()
            frontVelocity = frontvehicle.get_velocity()
            backVelocity = backvehicle.get_velocity()
            eDistance = backLocation.distance(frontLocation)
            eVelocity = (frontVelocity - backVelocity).length()

            #updating controller 
            throttle = pd_controller(eDistance, eVelocity, desired_distance, kp, kd)            
            steering = calculate_steering(backvehicle, frontLocation)              
            control = carla.VehicleControl(throttle=max(0, throttle), brake=abs(min(0, throttle)), steer=steering)
            backvehicle.apply_control(control)
            print(steering)
            '''
            if steering<0:
                controller.LogiPlayConstantForce(int(steering*100), 0) 
            elif steering>0:
                controller.LogiPlayConstantForce(0, int(steering*100)) 
            '''

    finally:
        controller.steering_shutdown()
        print('Destroying actors...')
        client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list])
        print('Done.')

if __name__ == '__main__':
    main()
