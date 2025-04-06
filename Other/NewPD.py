#!/usr/bin/env python

import glob
import os
import sys
#from telnetlib import TM
import carla
import random
import time
import math
import numpy as np
from agents.navigation.controller import VehiclePIDController
import threading
from collections import deque


#Global variables 
fps = 30 # Frames per second of simulation
max_simulation_time = 60  # Time for one lap
max_obstacles = 0 # Number of obstacles that will try to spawn 
debug = False # Change to show extra print statements 
distance_from_front = 5 # Distance between front vehicle  
frontspeed = 30 #Speed in kmh of front vehicle 
width = 800 #pygame screen width
height = 600 #pygame screen height
wheelcontrol = False #Set to True if using the steering wheel
datatracking = False #Set to True to record data 

# Create a global lock
lanechanging_lock = threading.Lock()

# Initialize Pygame
import pygame
pygame.init
pygame.font.init()
from pygame.locals import K_SPACE
screen = pygame.display.set_mode((width, height))
try:
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
except:
    print("no joystick")
################################################################################################################
#Custom Classes 
################################################################################################################

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

class VehiclePD:
    def __init__(self, vehicle, other_vehicle, world):
        self.vehicle = vehicle
        self.other_vehicle =other_vehicle
        self.world = world
        self.map = world.get_map()            
        self.kp_t = 0.1
        self.kd_t = 0.1
        self.ki_t = 0.0
        self.kd_s = 10.0
        self.kp_s = 0.1
        self.ki_s = 0.01
        self._dt = 0.03
        self._e_buffer = deque(maxlen=10)
        self.integral_throttle = 0
        self.desired_distance = distance_from_front # meters
        self.control = carla.VehicleControl(throttle=0.0, steer= 0.0, brake=0.0)

    def _get_waypoints_behind(self):
        front_waypoint = self.map.get_waypoint(self.other_vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)    
        waypoints_behind = []
        waypoints_behind = front_waypoint.previous(distance_from_front)
        if len(waypoints_behind) > 0:
            target_waypoint = waypoints_behind[0]
            return target_waypoint

    def _get_distance_error(self):
        target_waypoint = self._get_waypoints_behind()
        return (self.vehicle.get_transform().location.distance(target_waypoint.transform.location))

    def _get_velocity_error(self):
        return ((self.other_vehicle.get_velocity()-self.vehicle.get_velocity()).length())
        
         
    def _get_PD_throttle(self):
        eDistance = self._get_distance_error() 
        eVelocity = self._get_velocity_error()                                   
        error = eDistance - self.desired_distance 
        self.integral_throttle += error * self._dt
        throttle = max(0.0, min(self.kp_t * error + self.kd_t * eVelocity + self.ki_t * self.integral_throttle, 1.0))
        return(throttle)

    def _get_PD_steering(self):
        # Mainly taken from vehiclePIDcontroller code for steering
        transform = self.vehicle.get_transform()
        target_waypoint = self._get_waypoints_behind()
        v_begin = transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(transform.rotation.yaw)),
                                         y=math.sin(math.radians(transform.rotation.yaw)))        
        #vector in direction vehcile is facing 
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])

        #vector in direction from vehicle to target waypoint 
        w_vec = np.array([target_waypoint.transform.location.x - v_begin.x, 
                          target_waypoint.transform.location.y - v_begin.y, 0.0])

        #calculates angle between vectors
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        #Determines sign of angle 
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        steer = np.clip((self.kp_s * _dot) + (self.kd_s * _de / self._dt) + (self.ki_s * _ie * self._dt), -1.0, 1.0)
        return(steer) 
                

    def update(self):
        #updates throttle        
        throttle = self._get_PD_throttle()
        
        #print(throttle)
        if throttle >= 0:
            self.control.throttle = throttle 
        #if throttle <= 0:
            #self.control.brake = throttle
        #updates steering
        if wheelcontrol:
            steer = get_steering_input() # activated if using steering wheel
        else:
            steer = self._get_PD_steering()
            print(steer)

        self.control.steer = steer
            
        #apply control
        self.vehicle.apply_control(self.control)
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

def get_steering_input():
    pygame.event.pump()
    steering = joystick.get_axis(0)  # Axis 0 is typically the steering axis   
    return steering



def spawnObstacles(world, actor_list, spawn_points, obstacle_bp):
    num_obstacles = 0
    while num_obstacles < max_obstacles:
        random_spawn = random.choice(spawn_points)
        vehicles = world.get_actors().filter('vehicle.*')
        target_location = random_spawn.location
        
        map = world.get_map()

        target_waypoint1 = map.get_waypoint(target_location + carla.Location(x= 20 * math.cos(math.radians(random_spawn.rotation.yaw)) , y= 20 * math.sin(math.radians(random_spawn.rotation.yaw)) , z=0 ))
        target_waypoint2 = map.get_waypoint(target_location + carla.Location(x= -20 * math.cos(math.radians(random_spawn.rotation.yaw)) , y= -20 * math.sin(math.radians(random_spawn.rotation.yaw)) , z=0 ))
        
        no_corner = (target_waypoint1.road_id != 0) and (target_waypoint2.road_id != 0)
        

        min_distance = float('inf')



        for waypoint in map.generate_waypoints(2.0):
            if waypoint.is_junction:
                distance = target_location.distance(waypoint.transform.location)
                if distance < min_distance:
                    min_distance = distance
                    
        
        for vehicle in vehicles:
            # Calculate distance using CARLA's built-in distance function
            distance = target_location.distance(vehicle.get_location())
        
            if distance < min_distance:
                min_distance = distance
                            
        if min_distance > 40 and no_corner:
            obstacle  = world.try_spawn_actor(obstacle_bp[0], random_spawn)
            time.sleep(0.1)
            if obstacle:
                if debug:
                    print("obstacle spawned")               
                actor_list.append(obstacle)
                num_obstacles += 1
            else:
                if debug:
                    print("obstacle not spawned")
    else:
        if debug:
            print("all obstacles spawned")

def lanechanging(vehicle, custom_controller, world, tm):
    """
    Perform a smooth lane change to bypass an obstacle.
    """
   
    
    if debug:
        print("starting turn")
    vehicle.set_autopilot(False)
    # Get current transform (position and orientation) of the vehicle
    current_transform = vehicle.get_transform()
    current_location = current_transform.location
    current_rotation = current_transform.rotation
                
    # Calculate target location for bypassing the obstacle
    # Assuming a lateral shift of 3.5 meters (typical lane width)
    locations = []
    target_location0 = carla.Location(
        x=current_location.x + 5 * math.cos(math.radians(current_rotation.yaw)) ,  
        y=current_location.y + 5 * math.sin(math.radians(current_rotation.yaw)),  
        z=current_location.z
    )
    locations.append(target_location0)
    target_location1 = carla.Location(
        x=current_location.x + 18 * math.cos(math.radians(current_rotation.yaw)) + 3 * math.sin(math.radians(current_rotation.yaw)) ,  
        y=current_location.y - 3 * math.cos(math.radians(current_rotation.yaw)) + 18 * math.sin(math.radians(current_rotation.yaw)),  
        z=current_location.z
    )
    locations.append(target_location1)
    target_location2 = carla.Location(
        x=current_location.x + 28 * math.cos(math.radians(current_rotation.yaw)) + 3 * math.sin(math.radians(current_rotation.yaw)) ,  
        y=current_location.y - 3 * math.cos(math.radians(current_rotation.yaw)) + 28 * math.sin(math.radians(current_rotation.yaw)),  
        z=current_location.z
    )
    locations.append(target_location2)
    target_location3 = carla.Location(
        x=current_location.x + 40 * math.cos(math.radians(current_rotation.yaw)) ,  
        y=current_location.y + 40 * math.sin(math.radians(current_rotation.yaw)),  
        z=current_location.z
    )
    locations.append(target_location3)
        


    map = world.get_map()

    for index, item in enumerate(locations):
        while True:
            target_waypoint = map.get_waypoint(locations[index], project_to_road=True, lane_type=carla.LaneType.Driving)
            current_transform = vehicle.get_transform()
            current_location = current_transform.location
            
            # Calculate lateral error (distance to target y-coordinate)
            lateral_error = locations[index].y - current_location.y
            long_error = locations[index].x - current_location.x
            if abs(lateral_error) < 2 and abs(long_error) < 2:  # Threshold for stopping lane change
                if debug:
                    print(f"reached destination {index}")
                break

            control = custom_controller.run_step(frontspeed, target_waypoint)
            #print(control)
            vehicle.apply_control(control)
            
    vehicle.set_autopilot(True)
    

def handle_obstacle(vehicle, data, controller, world, sensor, tm):
    """
    Handle detected obstacles and trigger lane change.
    """
    

    #print(data)
    #print(data.other_actor.type_id)
    with lanechanging_lock:
        if data.other_actor.type_id.startswith('vehicle'):        
            #print(f"Obstacle detected: {data}")
            sensor.stop()
            lanechanging(vehicle, controller, world, tm)
            sensor.listen(lambda data: handle_obstacle(vehicle, data, controller, world, sensor, tm))

def updateSpectator(vehicle, spectator):
    vehicle_transform = vehicle.get_transform()
    forward_vector = vehicle_transform.get_forward_vector()
    right_vector = vehicle_transform.get_right_vector()
    
     #Uncomment for 3rd person
    # Calculate camera position behind the vehicle
    camera_location = vehicle_transform.location - 10 * forward_vector
    camera_location.z += 5  # Raise the camera slightly
        
    # Calculate camera rotation to look at the vehicle
    camera_rotation = vehicle_transform.rotation
    camera_rotation.pitch -= 15  # Tilt the camera down slightly
    '''   
    #Uncomment for 1st person
    # Calculate camera position behind the vehicle
    camera_location = vehicle_transform.location  + 0.4 * forward_vector - 0.4 * right_vector
    camera_location.z += 1.25  # Raise the camera slightly    
    # Calculate camera rotation to look at the vehicle
    camera_rotation = vehicle_transform.rotation
    '''
    # Set spectator's transform
    camera_transform = carla.Transform(camera_location, camera_rotation)
    spectator.set_transform(camera_transform)
    
def writeText(text):
    screen.fill((0,0,0))
    font = pygame.font.Font(None, 200)
    text_surface = font.render(text, True, (255,255,255))
    translucent_surface = pygame.Surface((text_surface.get_width(), text_surface.get_height()), pygame.SRCALPHA)
    translucent_surface.blit(text_surface, (0, 0))
    translucent_surface.set_alpha(128)
    text_rect = translucent_surface.get_rect(center=(width / 2, height / 2))
    screen.blit(translucent_surface, text_rect)
    pygame.display.flip()


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
                        
        #configure traffic manager 
        tm = client.get_trafficmanager(8000)
        #tm.set_synchronous_mode(True)
                        

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
#Actor Spawning 
################################################################################################################

        # Spawn the first vehicle at the random spawn point
        random_spawn_point = random.choice(spawn_points)
        frontvehicle = world.try_spawn_actor(frontvehicle_bp[0], random_spawn_point)

        if frontvehicle:
            if debug:
                print(f"Front Vehicle spawned at {random_spawn_point.location}")
            actor_list.append(frontvehicle)
    
            # Calculate a new spawn point 10 units behind the first vehicle
            transform_behind = carla.Transform(
                carla.Location(
                    x=random_spawn_point.location.x - distance_from_front * random_spawn_point.rotation.get_forward_vector().x,
                    y=random_spawn_point.location.y - distance_from_front * random_spawn_point.rotation.get_forward_vector().y,
                    z=random_spawn_point.location.z
                ),
                random_spawn_point.rotation
            )
    
            # Spawn the second vehicle at the calculated location
            backvehicle = world.try_spawn_actor(backvehicle_bp[0], transform_behind)
    
            if backvehicle:
                if debug:
                    print(f"Back Vehicle spawned at {transform_behind.location}")
                actor_list.append(backvehicle)
            else:
                if debug:
                    print("Retrying Spawn")
                frontvehicle.destroy()
                new_transform = carla.Transform(
                    carla.Location(
                        x=random_spawn_point.location.x + distance_from_front * random_spawn_point.rotation.get_forward_vector().x,
                        y=random_spawn_point.location.y + distance_from_front * random_spawn_point.rotation.get_forward_vector().y,
                        z=random_spawn_point.location.z
                    ),
                    random_spawn_point.rotation
                )
                frontvehicle = world.try_spawn_actor(frontvehicle_bp[0], new_transform)

                if frontvehicle:
                    if debug:
                        print(f"Front Vehicle respawned at {random_spawn_point.location}")
                    actor_list.append(frontvehicle)
    
                    # Spawn the second vehicle at the calculated location
                    backvehicle = world.try_spawn_actor(backvehicle_bp[0], random_spawn_point)
    
                    if backvehicle:
                        if debug:
                            print(f"Back Vehicle spawned at {transform_behind.location}")
                        actor_list.append(backvehicle)
                    else:
                        if debug:
                            print("Back vehicle still couldnt spawn")

        else:
            if debug:
                print("Failed to spawn front vehicle, Try again.")

        #spawns all obstacles
        spawnObstacles(world, actor_list, spawn_points, obstacle_bp)

        #Initialises sensor
        obstacle_bp = world.get_blueprint_library().find('sensor.other.obstacle')
        obstacle_bp.set_attribute('distance', '20')  # Detection distance in meters
        obstacle_bp.set_attribute('hit_radius', '0.5')  # Radius of the detection area
        obstacle_bp.set_attribute('only_dynamics', 'True')  # To detect only dynamic objects like vehicles
        obstacle_bp.set_attribute('debug_linetrace', 'False')  # Set to 'True' for visualization
        obstacle_bp.set_attribute('sensor_tick', '0.5')  # Set to 'True' for visualization


        # Attach the sensor to the ego vehicle
        sensor_transform = carla.Transform(carla.Location(x=2.0, z=1.0))  # Adjust position as needed   
        obstacle_sensor = world.spawn_actor(obstacle_bp, sensor_transform, attach_to=frontvehicle)
        actor_list.append(obstacle_sensor)
            
        spec_transform = carla.Transform(
                location = carla.Location(x = transform_behind.location.x - 10*transform_behind.rotation.get_forward_vector().x, y = transform_behind.location.y - 10 * transform_behind.rotation.get_forward_vector().y, z = transform_behind.location.z + 5),
                rotation = transform_behind.rotation 
                )
        spectator.set_transform(spec_transform)

################################################################################################################
#Vehicle Control
################################################################################################################
        tm.set_desired_speed(frontvehicle, frontspeed)
        tm.distance_to_leading_vehicle(frontvehicle,0)
        tm.ignore_lights_percentage(frontvehicle, 100)
        tm.ignore_signs_percentage(frontvehicle, 100)
            
        
            

        

        front_controller = VehiclePIDController(
        frontvehicle,
        args_lateral={'K_P': 2.0, 'K_D': 1.0, 'K_I': 0.0},
        args_longitudinal={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}
        )

        obstacle_sensor.listen(lambda data: handle_obstacle(frontvehicle, data, front_controller, world, obstacle_sensor, tm))

        #countdown 
        writeText("3")
        time.sleep(1)
        writeText("2")
        time.sleep(1)
        writeText("1")
        time.sleep(1)
        writeText("start!")
        time.sleep(1)
        screen.fill((0,0,0))
        pygame.display.flip()

        frontvehicle.set_autopilot(True)

        F_monitor = VehicleMonitor(frontvehicle, world)
        B_monitor = VehicleMonitor(backvehicle, world)
        B_controller = VehiclePD(backvehicle, frontvehicle, world)
                        

################################################################################################################
#main loop
################################################################################################################
       
        while True:
            world.tick()
            #pygame.display.flip()
            clock.tick(fps)   
                               
                            

            F_monitor.update()
            B_monitor.update()
            B_controller.update()
            
            
            

            updateSpectator(backvehicle, spectator)

            reset = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        reset = True
            if reset:
                print("Resetting Simulation")
                print("Press Space again to intialise")
                break
            # Add a small delay to avoid overwhelming the simulator
            world.wait_for_tick()
    
            

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
            
        

print("Press Space to intialise")
while True:    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                if __name__ == '__main__':
                    main()

