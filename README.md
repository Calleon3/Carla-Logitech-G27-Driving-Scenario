Cara and Logitech wheel driving scenario for testing haptic feedback and third eye technologies:

Introduction:
This project was developed as an undergraduate project so excuse the coding quality.
It aims to make a hardware-in-the-loop driving scenario inside of CARLA that allows a vehicle to be controlled through the use of a Logitech G27 steering wheel controller. The scenario is a car following scenario that involves avoiding obstacles. The scenario allows for the toggling of haptic feedback and third eye assistance. 

All code is in python and the project uses both LogiDrivePy and Pygame to interact with the steering wheel through the python script.

Supported devices:
This code was tested with the Logitech G27 gaming steering wheel but as it mainly uses very standard functions from Pygame and LogiDrivePy it may work with these devices aswell:
G29
G920
Driving Force GT
G27
G25
Driving Force Pro
MOMO Force
MOMO Racing
Formula Force GP
Driving Force
Formula Force

Dependencies:
This project has a lot of dependencies
The version of software used is inside brackets. For most of these a newer version of the software will also work however for python this version and any versions supported on the CARLA documentation website must be used.
Carla (version 0.9.15) (windows install, NOT built from source)
Python (version 3.7.9) (make sure python is added to path)
Logitech Gaming Software (This is no longer available directly from Logitech and so must be installed from else where)
pip (version 20.3+)

pip install:
pygame
pygame numpy 
<carla-0.9.15-cp37-cp37m-win_amd64>.whl (this is the python wheel file for Carla)
logidrivepy
shapely 
pygetwindow


System Setting changes:
-Memory integrity in the settings must be toggled off (To disable core isolation, open the Windows Security app, navigate to the Device Security tab, and then toggle off the Memory Integrity option under Core Isolation Details.)

-Inside the Logitech gaming software turn off the toggle that allows the game to change device settings (as otherwise LogiDrivePy seems to set wheel range to 200 degrees instead of 900 degrees) 
The centering spring was also disabled for the testing as a new one is applied inside the code.

Usage:
The main python scripts used during the final testing of the project are stored in the folder "FinalPythonCodes"
They must be stored inside of {CARLA_0.9.15\PythonAPI\carla} to run without errors with their current implementation.
The computer being used must also have a stable internet connection and be up to the minimum specifications set out on the CARLA website.

This code has only been tested on CARLA Town01 and due to some hard coded variables in its current state will not work with other maps. The code does not directly set the map to Town01 (which is possible) but instead changes the default town in the set up files to Town01. Use this copy and paste to avoid manual changing :"[/Script/EngineSettings.GameMapsSettings] EditorStartupMap=/Game/Carla/Maps/Town01_Opt.Town01_Opt GameDefaultMap=/Game/Carla/Maps/Town01_Opt.Town01_Opt ServerDefaultMap=/Game/Carla/Maps/Town01_Opt.Town01_Opt GlobalDefaultGameMode=/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C GameInstanceClass=/Script/Carla.CarlaGameInstance TransitionMap=/Game/Carla/Maps/Town01_Opt.Town01_Opt GlobalDefaultServerGameMode=/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C"



Important:
The combination of a new software CARLA and an old gaming wheel the G27 causes some integration problems so certain unwanted behaviours of the code are as such:
-You should not click on any screen after the spacebar has been pressed for initialisation during a test. This will cause all force feedback to be disabled permanently through the wheel.
-The order you initialise Pygame and LogiDrivePy is important as some orders only allow for force feedback or the steering angle to be read. (This line of code is also crucial to allow both to function together "os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1' ")
-For some reason I had to use this code to use LogiDrivePy 
"logi_controller = LogitechController(dll_path)
logi_controller.steering_initialize()"
where the dll_path is the location of this file "LogitechSteeringWheel.dll" 
(this shouldn't be necessary according to LogiDrivePy)
-Inside of Carla it is hard to get vehicles under the control of an autopilot or agent to change lanes as the PID's use the waypoints direction and will usually just turn around as soon as they realise they are in the other lane. So this code uses a manual overtake function. This should be changed if possible as if obstacles spawn to close to each other, a junction or a corner the front vehicle will drive off road or collide with something.
-The way this code handles the cameras in CARLA used for the main screen and third eye is very resource consuming. This means when the third eye is active (although set to a low frame rate) the main loop will lopp significantly slower causing the autopilot to update slower. This causes collisions and erroneous behaviour and so if in the future more cameras are to be used the data handling on the cameras should be improved or a better graphics card used.


(In the file "Test Codes" is all test scripts used to get to the final code. These are not fully commented and not named with ease of use in mind but there may be some useful information. Also some of the test scripts use software not listed on the dependencies.)

Final remarks:
This was a third year project and so no more updates can be expected for this project. I will try to assist anyone who may have similar issues with the code but response time may be slow.
I had a difficult time getting all of this code to function and it uses lots of work arounds but I thought it may help anyone doing a similar thing.


