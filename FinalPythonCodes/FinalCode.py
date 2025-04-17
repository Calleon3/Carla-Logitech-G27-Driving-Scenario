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

os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1' #need for pygame and logidrivepy to function at same time 

################################################################################################################
#Global variables
################################################################################################################
 
fps = 30 # Frames per second of simulation
max_obstacles = 30 # Number of obstacles that will try to spawn (limit is about 20-25)
distance_from_front = 6 # Distance the PID will attempt to keep between cars (over 10 can cause weird behaviour with junctions) 
# If using van must have atleast 6 meters for vehicles to spawn
frontspeed = 30 #Speed in kmh of front vehicle (going to fast may break the PID)
haptic_strength = 30 # 30 weak, 50 med ,70 strong, 100 full haptic control  
TIMEOUT = 120*1000  # Time in seconds * 1000 = milliseconds 

debug = True # Change to show extra print statements 
wheelcontrol = False #Set to True if using the steering wheel
haptic_feedback = False #Turn on to use haptic feedback (wheel control must be set to true)
camera_view = False #Turn on to stream camera 
third_eye_view = False #Turn on to use third eye
#reverse_haptic = False #Can only use this or 'wheelcontrol' 
Normal_Front_Vehicle = True # True = car, False = van

#Data recording
file_name = "Candidate1"
block_number = 1
trial_number = 1

duration = 0
Collision_count = 0
Collision_time = 0
Resets = 0
Offset = 0
Offset_count = 0
Error = 0
Error_count = 0
Overtakes = 0
num_obstacles = 0

if camera_view:
    width = 1920 #pygame screen width
    height = 1080 #pygame screen height
else:
    width = 800  #pygame screen width
    height = 600 #pygame screen height

third_width = 600
third_height = 400
third_eye_x = 0
third_eye_y = 0

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
frontID = ""

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
            if self.front:
                self.vehicle.set_autopilot(False)
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                self.other_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                time.sleep(0.5)
                self.vehicle.set_transform(waypoint.transform)
                waypoints_ahead = waypoint.previous(distance_from_front+1)
                wp = waypoints_ahead[0]
                self.other_vehicle.set_transform(wp.transform)
                self. vehicle.set_autopilot(True)
            else:         
                self.other_vehicle.set_autopilot(False)
                self.other_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                time.sleep(0.5)
                self.vehicle.set_transform(waypoint.transform)
                waypoints_ahead = waypoint.next(distance_from_front+1)
                wp = waypoints_ahead[0]
                self.other_vehicle.set_transform(wp.transform)
                self.other_vehicle.set_autopilot(True)
      

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
        
        global Offset,Offset_count, Error, Error_count 
        # Work out x or y differnece instead of just using distacne between target and current
        if abs(math.cos(math.radians(backvehicle.get_transform().rotation.yaw))) >= abs(math.sin(math.radians(backvehicle.get_transform().rotation.yaw))):
            dist = abs(target_waypoint.transform.location.x - back_location.x)
            Error += abs(target_waypoint.transform.location.x - back_location.x)
            Error_count += 1
            Offset += abs(target_waypoint.transform.location.y - back_location.y)
            Offset_count +=1
        else:
            dist = abs(target_waypoint.transform.location.y - back_location.y) 
            Error += abs(target_waypoint.transform.location.y - back_location.y) 
            Error_count += 1
            Offset += abs(target_waypoint.transform.location.x - back_location.x)
            Offset_count +=1
        if dist < 0.5:
            dist = 0.5
        # Calculate desired speed based on distance to target waypoint 
        desired_speed = max(0, dist) * frontspeed/4 # Converts distance error to desired speed using proportional control 
        #Sets max speed to front vehicles speed + 5 
        if desired_speed > (frontspeed + 5):
            desired_speed = frontspeed + 5
        # Apply control to back vehicle using PID controller
        control = back_controller.run_step(desired_speed , target_waypoint)
        steering = control.steer 
        
        if wheelcontrol:
            if haptic_feedback:
                logi_controller.logi_update()
                logi_controller.play_spring_force(0,int(steering*80), haptic_strength , 20) 
            else:
                logi_controller.logi_update()
                logi_controller.play_spring_force(0,0,20,20)
            control.steer = get_steering_input() *0.8 # activate if using steering wheel
            #control.steer = min(get_steering_input() *1.09,1.0)

           
        
        backvehicle.apply_control(control)
        

def spawnObstacles(world, actor_list, spawn_points, obstacle_bp):
    #List of corner coordinates on Town01
    corners = [
        carla.Location(x=3, y=3, z=0),
        carla.Location(x=393, y=3, z=0),
        carla.Location(x=393, y=325, z=0),
        carla.Location(x=3, y=325, z=0),    
    ]

    global num_obstacles
    attempts = 1000 #number of times an obstacle is attempted to be spawned before deciding it can not be done
    while num_obstacles < max_obstacles: 
        random_spawn = random.choice(spawn_points)
        vehicles = world.get_actors().filter('vehicle.*')
        target_location = random_spawn.location       
        map = world.get_map()

        #Next sections of code check if a potential spawn is close to a vehicle, junction or corner 
        min_distance = float('inf')
        min_dist_corner = float('inf')
        for waypoint in map.generate_waypoints(2.0):
            if waypoint.is_junction:
                distance = target_location.distance(waypoint.transform.location)
                if distance < min_distance:
                    min_distance = distance
                    if min_distance < 40:
                        break

        for corner in corners:
            distance = target_location.distance(corner)
            if distance < min_dist_corner:
                min_dist_corner = distance
                if min_dist_corner < 30:
                        break
 
        for vehicle in vehicles:           
            distance = target_location.distance(vehicle.get_location())
        
            if distance < min_distance:
                min_distance = distance
                if min_distance < 40:
                        break
        #If spawn point was not near to corner an obstacle will be spawned                    
        if min_distance >= 40 and min_dist_corner >= 30:
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
    # Assuming a lateral shift of 3 meters (typical lane width)
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
    global Overtakes
    Overtakes += 1
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
    
    #Uncomment for 3rd person server view
    # Calculate camera position behind the vehicle
    camera_location = vehicle_transform.location - 10 * forward_vector
    camera_location.z += 5  # Raise the camera slightly
        
    # Calculate camera rotation to look at the vehicle
    camera_rotation = vehicle_transform.rotation
    camera_rotation.pitch -= 15  # Tilt the camera down slightly
    '''   
    #Uncomment for 1st person server view 
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
    #adds camera image to screen
    screen.blit(surface, (0,0))
    if third_eye_view:
        global third_eye_surface 
        #adds third eye camera image to screen
        screen.blit(third_eye_surface , (third_eye_x,third_eye_y))

    global WriteText
    global HUD_TEXT
    if WriteText:
        font = pygame.font.Font(None, 200)
        text_surface = font.render(HUD_TEXT, True, (255,255,255))
        translucent_surface = pygame.Surface((text_surface.get_width(), text_surface.get_height()), pygame.SRCALPHA)
        translucent_surface.blit(text_surface, (0, 0))
        translucent_surface.set_alpha(128)
        text_rect = translucent_surface.get_rect(center=(width / 2, height / 2))
        #adds text to screen
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

def collision_data(event, collision_list):
    global duration
    global Collision_time
    global Collision_count
    global frontID
    #does not track collisions with frontvehicle as theoretically they should not collide
    if event.other_actor.id == frontID:
        return
    if event.other_actor.type_id.startswith('vehicle'):
    #Won't count as a collision if it has already collided with this vehicle as when it collides with a vehicle it 'collides' every frame
        if (event.other_actor.id not in collision_list):
            if debug:
                print("Collision detected with Vehicle")
            collision_list.append(event.other_actor.id)
            Collision_count += 1
            duration = pygame.time.get_ticks()
        else:
            duration = pygame.time.get_ticks() - duration  
            Collision_time += duration

def record_data(current_time, start_time, backvehicle, frontvehicle):
    delta_t = (pygame.time.get_ticks()-current_time)/1000
    ref_time = (current_time-start_time)/1000
    b_x = backvehicle.get_location().x
    b_y = backvehicle.get_location().y   
    b_vel = backvehicle.get_velocity()
    b_sp =  ((b_vel.x**2 + b_vel.y**2 + b_vel.z**2)**0.5)*3.6
    f_x = frontvehicle.get_location().x
    f_y = frontvehicle.get_location().y
    f_vel = frontvehicle.get_velocity()
    f_sp = ((f_vel.x**2 + f_vel.y**2 + f_vel.z**2)**0.5)*3.6
    #print(f"{delta_t:.2f},{ref_time:.2f},{b_x:.2f},{b_y:.2f},{b_sp:.2f},{f_x:.2f},{f_y:.2f},{f_sp:.2f}")
    global file_name, block_number, trial_number
    with open(f"{file_name}.txt", "a") as file:
        file.write(f"{block_number},{trial_number},{delta_t:.2f},{ref_time:.2f},{b_x:.2f},{b_y:.2f},{b_sp:.2f},{f_x:.2f},{f_y:.2f},{f_sp:.2f}\n")
            
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
                x=random_spawn_point.location.x - (distance_from_front+1) * random_spawn_point.rotation.get_forward_vector().x,
                y=random_spawn_point.location.y - (distance_from_front+1) * random_spawn_point.rotation.get_forward_vector().y,
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
                    x=random_spawn_point.location.x + (distance_from_front+1) * random_spawn_point.rotation.get_forward_vector().x,
                    y=random_spawn_point.location.y + (distance_from_front+1) * random_spawn_point.rotation.get_forward_vector().y,
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
    global frontID
    frontID = frontvehicle.id

    #Initialises sensor
    obstacle_bp = world.get_blueprint_library().find('sensor.other.obstacle')
    obstacle_bp.set_attribute('distance', '20')  # Detection distance in meters
    obstacle_bp.set_attribute('hit_radius', '0.5')  # Radius of the detection area
    obstacle_bp.set_attribute('only_dynamics', 'True')  # To detect only dynamic objects like vehicles
    obstacle_bp.set_attribute('debug_linetrace', 'True')  # Set to 'True' for visualization
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
    if Normal_Front_Vehicle:
        third_eye_transform = carla.Transform(carla.Location(x=2,y=0, z=1.25), third_eye_rotation)
    else:
        third_eye_transform = carla.Transform(carla.Location(x=4,y=1, z=1.25), third_eye_rotation)
    third_eye_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    third_eye_bp.set_attribute('image_size_x', '400')
    third_eye_bp.set_attribute('image_size_y', '600')
    third_eye_bp.set_attribute('fov', '100')
    third_eye_bp.set_attribute('sensor_tick', '0.1')
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
        tm.ignore_vehicles_percentage(frontvehicle, 100)
        tm.update_vehicle_lights(frontvehicle, True)   
            

        back_controller = VehiclePIDController(
            backvehicle,
            args_lateral={'K_P': 1.0, 'K_D': 0.1, 'K_I': 0.01},
            args_longitudinal={'K_P': 1.0, 'K_D': 0.2, 'K_I': 0.01}
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
        if third_eye_view:
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

        
        frontvehicle.set_autopilot(True)

        F_monitor = VehicleMonitor(frontvehicle, backvehicle, world, True)
        B_monitor = VehicleMonitor(backvehicle, frontvehicle, world, False)
        start_time = pygame.time.get_ticks()                
        current_time = start_time
################################################################################################################
#main loop
################################################################################################################
        previous_dist = 0
        while True:
            world.tick()
            #pygame.display.flip()
            clock.tick(fps)  
            record_data(current_time, start_time, backvehicle, frontvehicle)
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
            print(f"Total obstacles spawned:{num_obstacles}")
            print(f"Collision count:{Collision_count}")
            print(f"Collision time:{Collision_time/1000000}")
            print(f"Resets:{Resets}")
            print(f"Lateral Tracking Error average:{Offset/Offset_count}")
            print(f"Longitudinal Tracking Error average:{Error/Error_count}")
            print(f"Overtakes:{Overtakes}")
            

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
              
               


