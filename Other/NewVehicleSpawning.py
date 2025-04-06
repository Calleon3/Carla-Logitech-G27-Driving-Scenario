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
from agents.navigation.controller import VehiclePIDController

#Global variables 
max_simulation_time = 60  # seconds


# Initialize Pygame
import pygame
pygame.init
from pygame.locals import K_w, K_a, K_s, K_d, K_SPACE

################################################################################################################
#Custom Classes 
################################################################################################################
'''
class EnhancedBehaviorAgent(BehaviorAgent):
    def __init__(self, vehicle, ignore_traffic_light=False, behavior='normal'):
        # Create opt_dict with required parameters
        opt_dict = {
            'ignore_traffic_light': ignore_traffic_light,
            'behavior': behavior
        }
        
        super().__init__(vehicle, opt_dict=opt_dict)
        self.original_lane = None
        self.lane_change_timer = 0
        self.current_maneuver = None

    def _is_vehicle_in_front(self, vehicle):
        """Check if a vehicle is in front of the ego vehicle"""
        ego_transform = self._vehicle.get_transform()
        other_transform = vehicle.get_transform()

        # Calculate vector between vehicles
        vector = other_transform.location - ego_transform.location
        ego_forward = ego_transform.get_forward_vector()

        # Check if the other vehicle is within 60 field of view in front
        return (
            vector.dot(ego_forward) > 0.5 and  # Cosine similarity threshold
            vector.length() < 20  # Distance threshold
        )

    def detect_stationary_obstacle(self):
        """Detects stationary vehicles in front using sensor data"""
        vehicle_list = self._world.get_actors().filter('vehicle.*')
        
        for vehicle in vehicle_list:
            if vehicle.id == self._vehicle.id:
                continue
                
            if self._is_vehicle_in_front(vehicle):
                if vehicle.get_velocity().length() < 0.1:  # Stationary threshold
                    return True
        return False

    def _lane_change_maneuver(self):
        current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
    
        # Check available lanes
        left_lane = current_waypoint.get_left_lane()
        right_lane = current_waypoint.get_right_lane()
    
        # Find valid target lane
        target_lane = None
        if left_lane and left_lane.lane_type == carla.LaneType.Driving:
            target_lane = left_lane
        elif right_lane and right_lane.lane_type == carla.LaneType.Driving:
            target_lane = right_lane
    
        if not target_lane:
            return False
        
        # Generate path using the agent's existing route system
        self.original_lane = current_waypoint
        plan = []
    
        # Create temporary route for lane change
        for i in range(20):  # Create 20 waypoint path
            target_lane = target_lane.next(1.0)[0]  # Get next waypoint
            plan.append(target_lane.transform.location)
    
        # Use the agent's internal route planning
        self._local_planner.set_global_plan(plan)
        self.current_maneuver = "CHANGING_LANE"
        return True    

    def run_step(self, debug=False):
        if self.current_maneuver is None and self.detect_stationary_obstacle():
            if self._lane_change_maneuver():
                print("Initiating lane change maneuver")
                
        elif self.current_maneuver == "CHANGING_LANE":
            self.lane_change_timer += 1
            if self.lane_change_timer > 180:  # 3 seconds at 60 FPS
                self.current_maneuver = None
                self.original_lane = None
                                       
        return super().run_step(debug=debug)
'''
class VehicleMonitor:
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.map = world.get_map()
        self.stationary_start = None
        self.has_triggered = False
        self.speed_threshold = 0.1  # m/s
        self.stationary_duration = 3  # seconds

    def _get_nearest_road_waypoint(self):
        return self.map.get_waypoint(
            self.vehicle.get_location(),
            project_to_road=True,
            lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk)
        )

    def _move_to_nearest_road(self):
        #waypoint = self._get_nearest_road_waypoint()
        waypoint = self.map.get_waypoint(self.vehicle.get_location(), project_to_road=True)
        if waypoint:
            self.vehicle.set_transform(waypoint.transform)
            

    def update(self):
        velocity = self.vehicle.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5

        if speed < self.speed_threshold:
            if self.stationary_start is None:
                self.stationary_start = time.time()
            elif not self.has_triggered and (time.time() - self.stationary_start >= self.stationary_duration):
                self._move_to_nearest_road()
                self.has_triggered = True
                return True
        else:
            self.stationary_start = None
            self.has_triggered = False
        return False

################################################################################################################
#Custom Functions
################################################################################################################
 
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

    return 0

################################################################################################################
#Init simulation
################################################################################################################

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
        spawn_points = world.get_map().get_spawn_points()
            
        #Setting Blueprints 
        
        frontvehicle_bp = blueprint_library.filter('vehicle.dodge.charger_2020')  
        backvehicle_bp = blueprint_library.filter('vehicle.dodge.charger_2020')  
        altbackvehicle_bp = blueprint_library.filter('vehicle.mercedes.sprinter')  
        obstacle_bp = blueprint_library.filter('vehicle.audi.a2')

       
################################################################################################################
#Vehicle Spawning 
################################################################################################################

        # Spawn the first vehicle at the random spawn point
        random_spawn_point = random.choice(spawn_points)
        frontvehicle = world.try_spawn_actor(frontvehicle_bp[0], random_spawn_point)

        if frontvehicle:
            print(f"Front Vehicle spawned at {random_spawn_point.location}")
            actor_list.append(frontvehicle)
    
            # Calculate a new spawn point 10 units behind the first vehicle
            transform_behind = carla.Transform(
                carla.Location(
                    x=random_spawn_point.location.x + 40 * random_spawn_point.rotation.get_forward_vector().x,
                    y=random_spawn_point.location.y + 40 * random_spawn_point.rotation.get_forward_vector().y,
                    z=random_spawn_point.location.z
                ),
                random_spawn_point.rotation
            )
    
            # Spawn the second vehicle at the calculated location
            backvehicle = world.try_spawn_actor(backvehicle_bp[0], transform_behind)
    
            if backvehicle:
                print(f"Back Vehicle spawned at {transform_behind.location}")
                actor_list.append(backvehicle)
            else:
                print("Retrying Spawn")
                frontvehicle.destroy()
                new_transform = carla.Transform(
                    carla.Location(
                        x=random_spawn_point.location.x + 10 * random_spawn_point.rotation.get_forward_vector().x,
                        y=random_spawn_point.location.y + 10 * random_spawn_point.rotation.get_forward_vector().y,
                        z=random_spawn_point.location.z
                    ),
                    random_spawn_point.rotation
                )
                frontvehicle = world.try_spawn_actor(frontvehicle_bp[0], new_transform)

                if frontvehicle:
                    print(f"Front Vehicle respawned at {random_spawn_point.location}")
                    actor_list.append(frontvehicle)
    
                    # Spawn the second vehicle at the calculated location
                    backvehicle = world.try_spawn_actor(backvehicle_bp[0], random_spawn_point)
    
                    if backvehicle:
                        print(f"Back Vehicle spawned at {transform_behind.location}")
                        actor_list.append(backvehicle)
                    else:
                        print("Back vehicle still couldnt spawn")

        else:
            print("Failed to spawn front vehicle, Try again.")
      
        spec_transform = carla.Transform(
                location = carla.Location(x = transform_behind.location.x - 10*transform_behind.rotation.get_forward_vector().x, y = transform_behind.location.y - 10 * transform_behind.rotation.get_forward_vector().y, z = transform_behind.location.z + 5),
                rotation = transform_behind.rotation 
                )
        spectator.set_transform(spec_transform)

################################################################################################################
#Vehicle Control
################################################################################################################

        frontagent = BehaviorAgent(frontvehicle, behavior='normal')
        '''
        frontagent = EnhancedBehaviorAgent(
            frontvehicle, 
            ignore_traffic_light=False,  # Explicitly set this if needed
            behavior='normal'
        )
        '''
        #backagent = BehaviorAgent(backvehicle, behavior="normal")
        monitor = VehicleMonitor(backvehicle, world)
        frontagent.set_target_speed(20)
        #backagent.set_target_speed(60)

        custom_controller = VehiclePIDController(
            backvehicle,
            args_lateral={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0},
            args_longitudinal={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}
        )

        # Set the simulation to synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        #sets all traffic lights to green for X seconds 
        for actor in actors:
            if isinstance(actor, carla.TrafficLight):
                actor.set_state(carla.TrafficLightState.Green)
                actor.set_green_time(1000.0)
       

################################################################################################################
#main loop
################################################################################################################

        while True:
            world.tick()
            #pygame.display.flip()
            clock.tick(30)    

            frontvehicle.apply_control(frontagent.run_step())
            #backvehicle.apply_control(backagent.run_step())

            front_waypoint = world.get_map().get_waypoint(frontvehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        
            # Get waypoints behind the front vehicle (maintain same lane)
            waypoints_behind = front_waypoint.previous(10)  # Get waypoints 10 meters behind
        
            if len(waypoints_behind) > 0:
                target_waypoint = waypoints_behind[0]  # Use the first waypoint behind
            
                # Calculate desired speed based on distance to target waypoint
                back_location = backvehicle.get_location()
                current_distance = back_location.distance(target_waypoint.transform.location)
                desired_speed = max(0, current_distance - 5) * 3.6  # Convert to km/h
            
                # Apply control to back vehicle using PID controller
                control = custom_controller.run_step(desired_speed, target_waypoint)
                #backvehicle.apply_control(control)

            if monitor.update():
                print("Vehicle stationary for 3 seconds")
    
            

################################################################################################################
#Shut down code 
################################################################################################################

    finally:
        try:
            print('Destroying actors...')        
            client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list if actor])
            #added if actor to ensure only valid actors are destroyed
        except:
            print("Client or world not initialized properly.")
        print('Done.')


if __name__ == '__main__':
    main()

