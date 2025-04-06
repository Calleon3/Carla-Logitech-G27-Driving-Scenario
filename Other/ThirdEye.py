#!/usr/bin/env python

import glob
import os
import sys
from tabnanny import check
import carla
import random
import time
import math
from networkx import number_of_nonisomorphic_trees
import numpy as np
from agents.navigation.controller import VehiclePIDController
from collections import deque
from logidrivepy import LogitechController

os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1' #need for pygame and logidrivepy to function at same time 

################################################################################################################
#Global variables
################################################################################################################
 
fps = 30 # Frames per second of simulation
max_obstacles = 0 # Number of obstacles that will try to spawn (limit is about 30-35)
distance_from_front = 6 # Distance the PID will attempt to keep between cars (over 10 can cause weird behaviour with junctions) 
# If using van must have atleast 6 meters for vehicles to spawn
frontspeed = 30 #Speed in kmh of front vehicle (going to fast may break the PID)
haptic_strength = 30 # 30 weak, 50 med ,70 strong, 100 full haptic control  
TIMEOUT = 600*1000  # Time in seconds * 1000 = milliseconds 

debug = True # Change to show extra print statements 
wheelcontrol = True #Set to True if using the steering wheel
camera_view = True #Turn on to stream camera 
reverse_haptic = False #Can only use this or 'wheelcontrol' 
Normal_Front_Vehicle = True # True = car, False = van


#Data recording
duration = 0
Collision_count = 0
Collision_time = 0
Off_road_count = 0
Off_road_time = 0
Resets = 0

if camera_view:
    width = 1920 #pygame screen width
    height = 1080 #pygame screen height
else:
    width = 800  #pygame screen width
    height = 600 #pygame screen height

third_width = 800
third_height = 600

# Initialize Pygame
import pygame
pygame.init
pygame.font.init()
from pygame.locals import K_SPACE
screen = pygame.display.set_mode((width, height))

#Pygame Global Variables 
HUD_TEXT = ""
WriteText = False
third_eye_surface = pygame.Surface((third_width, third_height))

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
    def __init__(self, vehicle, other_vehicle, world, front):
        self.vehicle = vehicle
        self.other_vehicle = other_vehicle 
        self.world = world
        self.map = world.get_map()
        self.stationary_start = None
        self.has_triggered = False
        self.speed_threshold = 0.3  # m/s
        self.stationary_duration = 3  # seconds
        self.front = front
    def _move_to_nearest_road(self):
        waypoint = self.map.get_waypoint(self.vehicle.get_location(), project_to_road=True)
        if waypoint:
                self.vehicle.set_transform(waypoint.transform)
        if self.front:
            waypoints_ahead = waypoint.previous(distance_from_front)
            wp = waypoints_ahead[0]
            self.other_vehicle.set_transform(wp.transform)
        else:           
            waypoints_ahead = waypoint.next(distance_from_front)
            wp = waypoints_ahead[0]
            self.other_vehicle.set_transform(wp.transform)
      

    def update(self):
        velocity = self.vehicle.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5

        if speed < self.speed_threshold:
            if self.stationary_start is None:
                self.stationary_start = time.time()
            elif not self.has_triggered and (time.time() - self.stationary_start >= self.stationary_duration):
                global Resets 
                Resets += 1
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

    all_traffic_lights = world.get_actors().filter('traffic.traffic_light')
    for traffic_light in all_traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)
    return 0

def get_steering_input():
    pygame.event.pump()
    steering = joystick.get_axis(0)  # Axis 0 is typically the steering axis   
    return steering

def check_dist(world, frontvehicle, backvehicle):
    front_waypoint = world.get_map().get_waypoint(frontvehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)   
    waypoints_behind = []
    waypoints_behind = front_waypoint.previous(distance_from_front)
    dist = float('inf')
    if len(waypoints_behind) > 0:
        back_location = backvehicle.get_location()
        #gets closest waypoint to back vehicle to avoid going down junctions
        for wp in waypoints_behind:
            if back_location.distance(wp.transform.location) < dist:
                dist = back_location.distance(wp.transform.location)
                target_waypoint = wp
        #target_waypoint = waypoints_behind[0]  # Use the first waypoint behind
            
        # Calculate desired speed based on distance to target waypoint       
        current_distance = back_location.distance(target_waypoint.transform.location)  
        return(current_distance)

def back_control_update(world, frontvehicle, backvehicle, back_controller, logi_controller, changing_lane):
    front_waypoint = world.get_map().get_waypoint(frontvehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)   
    waypoints_behind = []

    if changing_lane: # If front vehicle is in other lane need to swap logic 
        # Get waypoints 'behind' the front vehicle (maintain same lane)        
        waypoints_behind = front_waypoint.next(distance_from_front)    
    else:
        # Get waypoints 'behind' the front vehicle (maintain same lane)
        waypoints_behind = front_waypoint.previous(distance_from_front)
             
    dist = float('inf')
    if len(waypoints_behind) > 0:
        back_location = backvehicle.get_location()
        #gets closest waypoint to back vehicle to avoid going down junctions
        for wp in waypoints_behind:
            if back_location.distance(wp.transform.location) < dist:
                dist = back_location.distance(wp.transform.location)
                target_waypoint = wp
        #target_waypoint = waypoints_behind[0]  # Use the first waypoint behind
            
        # Work out x or y differnece instead of just using distacne between target and current
        if abs(math.cos(math.radians(backvehicle.get_transform().rotation.yaw))) >= abs(math.sin(math.radians(backvehicle.get_transform().rotation.yaw))):
            dist = abs(target_waypoint.transform.location.x - back_location.x)
        else:
            dist = abs(target_waypoint.transform.location.y - back_location.y)      
        # Calculate desired speed based on distance to target waypoint 
        desired_speed = max(0, dist) * 3.6 * frontspeed/10 # Convert to km/h and normalises for front car speed limit
        
        # Apply control to back vehicle using PID controller
        control = back_controller.run_step(desired_speed , target_waypoint)
        steering = control.steer 
        
        if wheelcontrol:
            logi_controller.logi_update()
            logi_controller.play_spring_force(0,int(steering*100), haptic_strength , 20) 
            control.steer = get_steering_input() *0.75 # activate if using steering wheel
        
        backvehicle.apply_control(control)
        

def spawnObstacles(world, actor_list, spawn_points, obstacle_bp):
    corners = [
        carla.Location(x=3, y=3, z=0),
        carla.Location(x=393, y=3, z=0),
        carla.Location(x=393, y=325, z=0),
        carla.Location(x=3, y=325, z=0),    
    ]

    num_obstacles = 0
    attempts = 1000
    while num_obstacles < max_obstacles:
        random_spawn = random.choice(spawn_points)
        vehicles = world.get_actors().filter('vehicle.*')
        target_location = random_spawn.location
        
        map = world.get_map()

        min_distance = float('inf')

        for waypoint in map.generate_waypoints(2.0):
            if waypoint.is_junction:
                distance = target_location.distance(waypoint.transform.location)
                if distance < min_distance:
                    min_distance = distance
                    if min_distance < 30:
                        break

        for corner in corners:
            distance = target_location.distance(corner)
            if distance < min_distance:
                min_distance = distance
                if min_distance < 30:
                        break
 
        for vehicle in vehicles:           
            distance = target_location.distance(vehicle.get_location())
        
            if distance < min_distance:
                min_distance = distance
                if min_distance < 30:
                        break
                            
        if min_distance >= 30 :
            obstacle  = world.try_spawn_actor(obstacle_bp[0], random_spawn)
            time.sleep(0.1)
            if obstacle:                             
                actor_list.append(obstacle)
                num_obstacles += 1
                attempts = 1000
                if debug:
                    print("obstacle spawned") 
                    print(num_obstacles)
            else:
                if debug:
                    print("obstacle not spawned")
        attempts -= 1
        if attempts == 0:
            break

    else:
        if debug:
            print("all obstacles spawned")

def lanechanging(vehicle, custom_controller, world):
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

    for index, items in enumerate(locations):    
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
   
    

def handle_obstacle(vehicle, data, front_controller, world, sensor):
    """
    Handle detected obstacles and trigger lane change.
    """
    
   
    #print(data)
    #print(data.other_actor.type_id)
    
    if data.other_actor.type_id.startswith('vehicle'):        
        #print(f"Obstacle detected: {data}")
        sensor.stop()
        lanechanging(vehicle, front_controller, world)       
        sensor.listen(lambda data: handle_obstacle(vehicle, data, front_controller, world, sensor))
    


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
    #Uncomment for 1st person from the spectator 
    # Calculate camera position behind the vehicle
    camera_location = vehicle_transform.location  + 0.4 * forward_vector - 0.4 * right_vector
    camera_location.z += 1.25  # Raise the camera slightly    
    # Calculate camera rotation to look at the vehicle
    camera_rotation = vehicle_transform.rotation
    '''
    # Set spectator's transform
    camera_transform = carla.Transform(camera_location, camera_rotation)
    spectator.set_transform(camera_transform)
    
def DisplayText (text):
    screen.fill((0,0,0))
    font = pygame.font.Font(None, 200)
    text_surface = font.render(text, True, (255,255,255))
    translucent_surface = pygame.Surface((text_surface.get_width(), text_surface.get_height()), pygame.SRCALPHA)
    translucent_surface.blit(text_surface, (0, 0))
    translucent_surface.set_alpha(128)
    text_rect = translucent_surface.get_rect(center=(width / 2, height / 2))
    screen.blit(translucent_surface, text_rect)
    pygame.display.flip()
    

def process_img(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array)
    surface = pygame.transform.flip(surface, True, False)  # Flip horizontally
    screen.blit(surface, (0,0))

    global third_eye_surface 
    screen.blit(third_eye_surface , (0,0))

    global WriteText
    global HUD_TEXT
    if WriteText:
        font = pygame.font.Font(None, 200)
        text_surface = font.render(HUD_TEXT, True, (255,255,255))
        translucent_surface = pygame.Surface((text_surface.get_width(), text_surface.get_height()), pygame.SRCALPHA)
        translucent_surface.blit(text_surface, (0, 0))
        translucent_surface.set_alpha(128)
        text_rect = translucent_surface.get_rect(center=(width / 2, height / 2))
        screen.blit(translucent_surface, text_rect)
    pygame.display.flip()

def process_img_third_eye(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array)
    surface = pygame.transform.flip(surface, True, False)  # Flip horizontally
    global third_eye_surface
    third_eye_surface = surface 
    #screen.blit(surface, (0,0))
    #pygame.display.flip()

def collision_data(event, collision_list):
    global duration
    global Collision_time
    global Collision_count
    if event.other_actor.type_id.startswith('vehicle'):
        if event.other_actor.type_id not in collision_list:
            if debug:
                print("Collision detected with Vehicle")
            collision_list.append(event.other_actor.type_id)
            Collision_count += 1
            duration = pygame.time.get_ticks()
        else:
            duration = pygame.time.get_ticks() - duration  
            Collision_time += duration
            
def spawn_vehicles(world, spawn_points, frontvehicle_bp, backvehicle_bp, actor_list, spectator):
    actors = []
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
        
    #Initialise Collision sensor
    collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_bp.set_attribute("role_name", "collision_sensor")
    transform = carla.Transform(carla.Location(x=0.0, z=0.0))
    collision_sensor = world.spawn_actor(collision_bp, transform, attach_to=backvehicle)
    actor_list.append(collision_sensor)

    #Initialises backcamera
    camera_rotation = carla.Rotation(pitch=0, yaw=0, roll=90)
    camera_transform = carla.Transform(carla.Location(x=0.2,y=-0.4, z=1.25), camera_rotation)
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1080')
    camera_bp.set_attribute('image_size_y', '1920')
    camera_bp.set_attribute('fov', '100')
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=backvehicle)
    actor_list.append(camera)

    #Third eye camera
    third_eye_rotation = carla.Rotation(pitch=0, yaw=0, roll=90)
    third_eye_transform = carla.Transform(carla.Location(x=0.2,y=-0.4, z=1.25), third_eye_rotation)
    third_eye_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    third_eye_bp.set_attribute('image_size_x', '600')
    third_eye_bp.set_attribute('image_size_y', '800')
    third_eye_bp.set_attribute('fov', '100')
    third_eye = world.spawn_actor(third_eye_bp, third_eye_transform, attach_to=frontvehicle)
    actor_list.append(third_eye)

    spec_transform = carla.Transform(
            location = carla.Location(x = transform_behind.location.x - 10*transform_behind.rotation.get_forward_vector().x, y = transform_behind.location.y - 10 * transform_behind.rotation.get_forward_vector().y, z = transform_behind.location.z + 5),
            rotation = transform_behind.rotation 
            )
    spectator.set_transform(spec_transform)
    actors.append(frontvehicle)
    actors.append(backvehicle)
    actors.append(obstacle_sensor)
    actors.append(collision_sensor)
    actors.append(camera)
    actors.append(third_eye)
    return actors


################################################################################################################
#Init simulation
################################################################################################################


def main():
    #defining variables 
    actor_list = []
    collision_list = []   
    
    try: 
        try:
            dll_path = "C:\Program Files\Logitech\Gaming Software\SDKs\LogitechSteeringWheel.dll"
            # Initialize the controller
            logi_controller = LogitechController(dll_path)
            logi_controller.steering_initialize()
        except:
            print("Haptics broke")
        #Joining server as client 
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        clock = pygame.time.Clock()
                        
        #configure traffic manager 
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
                        

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
        altfrontvehicle_bp = blueprint_library.filter('vehicle.mercedes.sprinter')  
        obstacle_bp = blueprint_library.filter('vehicle.audi.a2')

       
################################################################################################################
#Actor Spawning 
################################################################################################################

        if Normal_Front_Vehicle:
            actors = spawn_vehicles(world, spawn_points, frontvehicle_bp, backvehicle_bp, actor_list, spectator)
        else:
            actors = spawn_vehicles(world, spawn_points, altfrontvehicle_bp, backvehicle_bp, actor_list, spectator)
        frontvehicle = actors[0]
        backvehicle = actors[1]
        obstacle_sensor = actors[2]
        collision_sensor = actors[3]
        camera = actors[4]
        third_eye = actors[5]

        #spawns all obstacles
        spawnObstacles(world, actor_list, spawn_points, obstacle_bp)

################################################################################################################
#Vehicle Control
################################################################################################################ 
        
        tm.set_desired_speed(frontvehicle, frontspeed)
        tm.distance_to_leading_vehicle(frontvehicle,0)
        tm.ignore_lights_percentage(frontvehicle, 100)
        tm.ignore_signs_percentage(frontvehicle, 100)
        tm.update_vehicle_lights(frontvehicle, True)   
            

        back_controller = VehiclePIDController(
            backvehicle,
            args_lateral={'K_P': 1.0, 'K_D': 0.1, 'K_I': 0.01},
            args_longitudinal={'K_P': 1.0, 'K_D': 0.1, 'K_I': 0.01}
        )

        front_controller = VehiclePIDController(
        frontvehicle,
        args_lateral={'K_P': 2.0, 'K_D': 1.0, 'K_I': 0.0},
        args_longitudinal={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}
        )

        obstacle_sensor.listen(lambda data: handle_obstacle(frontvehicle, data, front_controller, world, obstacle_sensor))
        collision_sensor.listen(lambda event: collision_data(event, collision_list))
        if camera_view:
            camera.listen(process_img)
            third_eye.listen(process_img_third_eye)
        
        if wheelcontrol:
            #Centers the steering wheel
            logi_controller.logi_update()
            logi_controller.play_spring_force(0,0,100, 100)
            time.sleep(1)
            logi_controller.logi_update()
            logi_controller.stop_spring_force(0)
            #Adds a constant damping force to wheel
            logi_controller.logi_update()
            logi_controller.play_damper_force(0,40)



        #countdown 
        global WriteText
        global HUD_TEXT
        WriteText = True
        HUD_TEXT = "3"
        time.sleep(1)
        HUD_TEXT = "2"
        time.sleep(1)
        HUD_TEXT = "1"
        time.sleep(1)
        HUD_TEXT = "Start!"
        time.sleep(1)
        WriteText = False

        start_time = pygame.time.get_ticks()
        frontvehicle.set_autopilot(True)

        F_monitor = VehicleMonitor(frontvehicle, backvehicle, world, True)
        B_monitor = VehicleMonitor(backvehicle, frontvehicle, world, False)
                        

################################################################################################################
#main loop
################################################################################################################
        previous_dist = 0
        while True:
            world.tick()
            #pygame.display.flip()
            clock.tick(fps)  

            current_time = pygame.time.get_ticks()
            if current_time - start_time >= TIMEOUT:
                print("Test Over")
                break                   
            elif current_time - start_time >= TIMEOUT - 1000:
                WriteText = True
                HUD_TEXT = "Test Over"
            elif current_time - start_time >= TIMEOUT - 2000:
                WriteText = True
                HUD_TEXT = "1"
            elif current_time - start_time >= TIMEOUT - 3000:
                WriteText = True
                HUD_TEXT = "2"
            elif current_time - start_time >= TIMEOUT - 4000:
                WriteText = True
                HUD_TEXT = "3"

            '''
            print(frontvehicle.get_control().brake)
            vehicle_control = carla.VehicleControl()
            vehicle_control.brake = 0.0  
            frontvehicle.apply_control(vehicle_control)
            print(frontvehicle.get_control().brake)
            '''
            updated = F_monitor.update() or B_monitor.update()
            if updated:
                if wheelcontrol:
                    #Centers the steering wheel
                    logi_controller.logi_update()
                    logi_controller.stop_damper_force(0)
                    logi_controller.play_spring_force(0,0,100, 100)
                    time.sleep(1)
                    logi_controller.logi_update()
                    logi_controller.stop_spring_force(0)
                    #Adds a constant damping force to wheel
                    logi_controller.logi_update()
                    logi_controller.play_damper_force(0,40)

                current_dist = check_dist(world, frontvehicle, backvehicle)
                back_control_update(world, frontvehicle, backvehicle, back_controller, logi_controller, False)
                previous_dist = current_dist
            else:
                current_dist = check_dist(world, frontvehicle, backvehicle)
                if (current_dist - previous_dist) > 6:
                       back_control_update(world, frontvehicle, backvehicle, back_controller, logi_controller, True)
                else:
                    back_control_update(world, frontvehicle, backvehicle, back_controller, logi_controller, False)
                    previous_dist = current_dist

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
            print("Data Recorded:")
            print(f"Collision count:{Collision_count}")
            print(f"Collision time:{Collision_time/1000000}")
            print(f"Resets:{Resets}")

            print('Destroying actors...')        
            client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list if actor])
            #added if actor to ensure only valid actors are destroyed
            if wheelcontrol:
                logi_controller.stop_constant_force(0)
                logi_controller.stop_spring_force(0)
                logi_controller.stop_damper_force(0)
                logi_controller.steering_shutdown()
            pygame.quit()
            sys.exit(0)
        except:
            print("Client or world not initialized properly.")
        print('Done.')
            
        

print("Press Space to intialise")
DisplayText("Press Space to intialise")
while True:    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                DisplayText("Starting")
                if __name__ == '__main__':
                    main()  
              
               


