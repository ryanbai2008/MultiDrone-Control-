import pygame
import json
import math
import cv2 
import time
from time import sleep
import numpy as np
import threading
import sys
from mapBackground import Background
import map
from localizeIRT import localizer
import socket
import os
from customtello import VideoProxyServer
from customtello import myTello

import path_planner
import tello_tracking
import collision
import logging
import platform
import subprocess
import avoid
import re



# Define the IP addresses of the two Wi-Fi adapters
WIFI_ADAPTER_1_IP = "192.168.10.2"  # IP address of Wi-Fi Adapter 1 (connected to Drone 1)
WIFI_ADAPTER_2_IP = "192.168.10.3"  # IP address of Wi-Fi Adapter 2 (connected to Drone 2)

drone_ips = [WIFI_ADAPTER_1_IP, WIFI_ADAPTER_2_IP]
base_port = 30000
TELLO_IP = "192.168.10.1"

# Adapter names
WIFI_1 = "Wi-Fi"
WIFI_2 = "Wi-Fi 2"

def run_command(command):
    """Runs a system command and returns output."""
    try:
        result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        print(f"Error running command: {e}")
        return None

def get_connected_ssid(adapter):
    """Returns the SSID of the currently connected network for the given adapter."""
    system = platform.system()
    
    if system == "Windows":
        result = run_command('netsh wlan show interfaces')
        match = re.search(r"(?:IP Address|IPv4 Address)[\s.]+:\s+([\d.]+)", result)
        return match.group(1) if match else None
    elif system in ["Linux", "Darwin"]:
        result = run_command(f'nmcli -t -f active,ssid dev wifi | grep "^yes"')
        return result.strip().split(":")[-1] if result else None
    return None


def connect_wifi(adapter, ssid):
    """Connects Wi-Fi adapter to a specific Tello network only if not already connected."""
    current_ssid = get_connected_ssid(adapter)
    
    if current_ssid == ssid:
        print(f"{adapter} is already connected to {ssid}. Skipping connection.")
        return
    
    print(f"Connecting {adapter} to {ssid}...")
    system = platform.system()
    
    if system == "Windows":
        run_command(f'netsh wlan connect name="{ssid}" interface="{adapter}"')
    else:  # Linux/macOS
        run_command(f'nmcli device wifi connect "{ssid}" ifname {adapter}')
    
    time.sleep(2)



def get_current_ip(adapter):
    """Returns the current IP address of the Wi-Fi adapter."""
    system = platform.system()
    
    if system == "Windows":
        result = run_command(f'netsh interface ip show address name="{adapter}"')
        match = re.search(r"IP Address:\s+([\d.]+)", result)
        return match.group(1) if match else None
    elif system in ["Linux", "Darwin"]:
        result = run_command(f'ip addr show {adapter}')
        match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)/', result)
        return match.group(1) if match else None
    return None

def set_static_ip(adapter, ip):
    """Assigns a static IP to the Wi-Fi adapter only if it is not already set."""
    current_ip = get_current_ip(adapter)
    
    if current_ip == ip:
        print(f"Static IP for {adapter} is already {ip}. Skipping.")
        return
    
    print(f"Setting static IP {ip} for {adapter}...")
    system = platform.system()
    
    if system == "Windows":
        run_command(f'netsh interface ip set address name="{adapter}" static {ip} 255.255.255.0')
    else:  # Linux/macOS
        run_command(f'sudo ifconfig {adapter} {ip} netmask 255.255.255.0 up')


def check_route(ip):
    """Checks if a route already exists for the given IP."""
    system = platform.system()
    
    if system == "Windows":
        result = run_command("route print")
    elif system in ["Linux", "Darwin"]:
        result = run_command("ip route show")
    else:
        return False
    
    return ip in result if result else False

def add_route():
    """Adds routing rules only if they do not already exist for each adapter."""
    system = platform.system()

    route_1_exists = check_route(WIFI_ADAPTER_1_IP)
    route_2_exists = check_route(WIFI_ADAPTER_2_IP)

    if route_1_exists and route_2_exists:
        print(f"Routes for {TELLO_IP} already exist. Skipping.")
        return

    print(f"Adding missing route(s) for {TELLO_IP}...")

    if system == "Windows":
        if not route_1_exists:
            run_command(f'route -p add {TELLO_IP} mask 255.255.255.255 {WIFI_ADAPTER_1_IP} metric 1')
        if not route_2_exists:
            run_command(f'route -p add {TELLO_IP} mask 255.255.255.255 {WIFI_ADAPTER_2_IP} metric 1')
    elif system in ["Linux", "Darwin"]:
        if not route_1_exists:
            run_command(f'sudo ip route add {TELLO_IP} via {WIFI_ADAPTER_1_IP} dev wlan0')
        if not route_2_exists:
            run_command(f'sudo ip route add {TELLO_IP} via {WIFI_ADAPTER_2_IP} dev wlan1')

# Connect to Tello networks
connect_wifi(WIFI_1, "TELLO-D06F9F")
connect_wifi(WIFI_2, "TELLO-EE4263")

# Assign static IPs
set_static_ip(WIFI_1, WIFI_ADAPTER_1_IP)
set_static_ip(WIFI_2, WIFI_ADAPTER_2_IP)

# Add route if necessary
add_route()

# Add routing rules
#add_route()

#proxy_server = VideoProxyServer(drone_ips, server_ip, base_port)
#proxy_server.start_proxy()

drone1 = myTello(WIFI_ADAPTER_1_IP, base_port)

drone1.connect()



def start_keep_alive():
    keep_alive_thread = threading.Thread(target=drone1.keep_alive, daemon=True)
    keep_alive_thread.start()
    return keep_alive_thread
keep_alive_thread = start_keep_alive()

battery1 = drone1.getBattery()

height1 = drone1.getHeight()

print(battery1)

def updateScreen():
    startingyaw1 = 0
    startingyaw2 = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False    
        speedx1 = 10
        speedx2 = 10
        speedz1  = drone1.get_AngularSpeed(startingyaw1)

        battery1 = drone1.getBattery()

        height1 = drone1.getHeight()

        startMap.start_screen(battery1, speedx1, speedz1, height1, 0, 0, 0, 0)
        sleep(10)

#creates a map of the environment on pygame
pygame.init()
screen = pygame.display.set_mode([864, 586])
screen_width, screen_height = pygame.display.get_surface().get_size()
pygame.display.set_caption("Path Planning with Map (BRViz)")
screen.fill((255, 255, 255))

speedx1 = drone1.get_speed()
speedz1 = drone1.get_AngularSpeed(0)
battery1 = drone1.getBattery()
height1 = drone1.getHeight()

isRunning = True
sizeCoeff = 400/42 # actual distance/pixel distance in cm (CHANGE THIS VALUE IF YOUR CHANGING THE MAP)

def scaleImgDown(img, scale_factor):
    original_width, original_height = img.get_size()
    new_width = int(original_width * scale_factor)
    new_height = int(original_height * scale_factor)
    img = pygame.transform.smoothscale(img, (new_width, new_height))# Scale the image
    return img

startMap = map.initializeMap(screen, "Make Drone 1 Path")
startMap.start_screen(battery1, speedx1, speedz1, height1, 0, 0, 0, 0)

#DRONE 1 MAPPING
map1 = map.mapStart(sizeCoeff, screen, Background('mymap.png', [0, 105], 0.7))
angle, distanceInCm, distanceInPx, path = map1.createMap()
pygame.draw.line(screen, (0, 0, 0), path[1], path[2], 6) #creates a line as the edges                
pygame.draw.circle(screen, (0, 0, 255), path[1], 5) #To note where the nodes are
pygame.draw.circle(screen, (0, 0, 255), path[2], 5) #To note where the nodes are

# Define the area you want to save (x, y, width, height)
saveImg = pygame.Rect(0, 105, screen_width, screen_height-105)
# Create a new Surface to store the part of the screen
path1img = screen.subsurface(saveImg).copy()
pygame.image.save(path1img, "pathPlanned.png") #Saves new background with path

startMap.changeInstruction("Make Drone 2 Path")
startMap.start_screen(battery1, speedx1, speedz1, height1, 0, 0, 0, 0)

#DRONE 2 Mapping
startMap.changeInstruction("Add the Subject")

startMap.start_screen(battery1, speedx1, speedz1, height1, 0, 0, 0, 0)
pygame.display.update()

personx, persony, personpospx = map1.addPerson(sizeCoeff)
personpos = (personx, persony)

print("HELLO")
startMap.changeInstruction("Moving Drones...")
startMap.start_screen(battery1, speedx1, speedz1, height1, 0, 0, 0, 0)

print(angle)
print(distanceInCm)
print(distanceInPx)
print(path)

point1 = path[0] 
point2 = path[1] 

#Saves the screen to be blitted
saveImg = pygame.Rect(0, 100, screen_width, screen_height-105)
# Create a new Surface to store the part of the screen
path1img = screen.subsurface(saveImg).copy()
pygame.image.save(screen, "pathPlanned2.png") #Saves new background with path


#Makes the drone image on a path
drone1Img = pygame.image.load('tello3.png')  # Replace with your image file path
drone1Img = scaleImgDown(drone1Img, 0.085) #Scale down to 8.5%


#position values for path planning
start_pos1X, start_pos1Y = path[0]
end_pos1X, end_pos1Y = path[1]

#drones current position values
drone_1_pos = [start_pos1X, start_pos1Y, 0] #initial angle of 0
drone_1_movement = [0, 0]


#path planning objects and CV objects
drone_1_path_plan = path_planner.PathPlan(start_pos1X, end_pos1X, start_pos1Y, end_pos1Y, drone_1_pos[2]) #########place both drones facing to the right facing horizontally
drone1_CV = tello_tracking.CV()

#has goal been reached for drones
drone_1_terminate = False


#how frequently the position is updated
sleep_time = 0
timer = 0
iter = 0

def drawPoints(screen, points, droneimg, yaw):
    font = pygame.font.SysFont('Times',25)
    
    for point in points:
        pygame.draw.circle(screen, (255, 0, 0), point, 3) #draws a red dot/line for the visited nodes/area
    # Rotate the image based on the yaw angle and draw it on the screen
    rotated_image = pygame.transform.rotate(droneimg, -yaw)
    image_rect = rotated_image.get_rect(center=(points[-1][0], points[-1][1]))
    screen.blit(rotated_image, image_rect.topleft)

    pygame.draw.circle(screen, (0, 255, 0), points[-1], 3) #green dot on the image for tracking
    
    #adds positional text data, (0,0) is bottom left corner
    x_cord = (int)((points[-1][0])/sizeCoeff)
    y_cord  = (int)((screen_height  - points[-1][1])/sizeCoeff)
    position_text = font.render(f'({x_cord}, {y_cord})cm', True, (255, 0, 0))
    screen.blit(position_text, (points[-1][0] + 10, points[-1][1] + 10))

def localize():
    background = Background('pathPlanned2.png', [0, 0], 1)
    x,y = 0, 0 # origin

    drone1points = [(x, y)]
    drone2points = [(x, y)]

    updateTime = 0.004 #amount of times we want to step (CHANGE THIS VALUE DEPENDING ON ACCCURACY, Controls odometry accuracy)
    angleUpdateTime = 0.005
    step1 = 0
    yaw1 = 0
    step2 = 0
    yaw2 = 0

    start_pos1 = path[0]
    end_pos1 = path[1]

    # Initialize the current position
    drone1current_pos = list(start_pos1)
    drone1previous_pos = drone1current_pos.copy()

    linearSpeed = 500
    angularSpeed = 500

    timeDur = distanceInCm/linearSpeed
    rotationDur = angle/angularSpeed


    drone1num_steps = int(timeDur / updateTime)
    drone1angle_num_steps = abs(int(rotationDur / angleUpdateTime))

    drone2previous_pos= drone2current_pos.copy()
  

    # Calculate the increments in x and y directions
    dx1 = (end_pos1[0] - start_pos1[0]) / drone1num_steps
    dy1 = (end_pos1[1] - start_pos1[1]) / drone1num_steps

   
    initial_yaw = 0  # Initial yaw angle in 
    target_yaw1 = map1.get_angle(path[0], personpospx, (path[0][0], path[0][1]+10))
    print(target_yaw1)

    drone1points.append(drone1current_pos)
    drone2points.append(drone2current_pos)

    #start_time = pygame.time.get_ticks()  # Get the start time
    #rotation_done = False
    drawPoints(screen, drone1points, drone1Img, yaw1)

    #MOVES THE DRONES
    #drone_thread = threading.Thread(target=move_tello, args={distanceInCm, distanceInCm2, angle, angle2})
    #drone_thread.start()

    #updates the screen
   
    #delays for the takeoff time
    time.sleep(5)
    running = True
    rotating1 = True
    rotating2 = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False    

        screen.blit(background.image, (0, 0))    

        if rotating1:
            if yaw1 != target_yaw1:
                yaw1 += (((target_yaw1) - initial_yaw) / drone1angle_num_steps)
                if abs(yaw1 - target_yaw1) < 0.1:
                    yaw1 = target_yaw1  # Snap to target yaw if close
                    rotating1 = False
        else:
            yaw1 = map1.get_angle(drone1current_pos, personpospx, (drone1current_pos[0], drone1current_pos[1]+10))
            if step1 <= drone1num_steps:
                previous_pos1 = drone1current_pos.copy()
                drone1current_pos[0] += dx1
                drone1current_pos[1] += dy1
                drone1points.append(tuple(drone1current_pos))
                # Increment yaw angle
                drone1points.append((int(drone1current_pos[0]), int(drone1current_pos[1])))
                step1 += 1

        # Draw the frame
        drawPoints(screen, drone1points, drone1Img, yaw1)

        pygame.display.update()
        
        pygame.time.delay(int(updateTime*1000))


    pygame.quit()
    sys.exit()

    # Ensure the final position is exactly the end position
    drone1current_pos = end_pos1
    drone1points.append(tuple(drone1current_pos))

    drone2current_pos = end_pos2
    drone2points.append(tuple(drone2current_pos))

    pygame.display.flip()

localizeThread = threading.Thread(target=localize)
localizeThread.start()
screenThread = threading.Thread(target=updateScreen)
screenThread.start()

try:
    #turn on drones cameras
    drone1.streamon()
    drone1.start_video_thread(1)
    drone1.takeoff()

    #path
    start_1_X, start_1_Y, end_1_X, end_1_Y = path[0][0], path[0][1], path[1][0], path[1][1]
    path1 = [start_1_X, start_1_Y, end_1_X, end_1_Y]

    #other drone path

    #drone current values
    drone_1_pos = [path1[0], path1[1], 0] #(X, Y, angle), STARTING ANGLE MUST BE 0 DEGREES

    #drone movement
    drone_1_movement = [0, 0, 0] #(delta X, delta Y, delta angle)

    #path planning and CV and collision objects
    drone_1_path_plan = path_planner.PathPlan(path1[0], path1[2], path1[1], path1[3], drone_1_pos[2])
    drone_1_CV = tello_tracking.CV()

    #goal reached for drones?
    drone_1_terminate = False

    #turn on drone

    drone1.send_rc(0, 0, 60, 0)

    #timer for position updates
    sleep_time = 0
    timer = 0
    iter = 0

    #drone heights
    time.sleep(1)
    drone_height = 200
    normal_height = 200
    while drone1.getHeight() < normal_height:
        drone1.send_rc(0, 0, 0, 20)
    drone1.send_rc(0, 0, 0, 0)
    go_up = 0

    #total time elapsed
    start_time = time.time()
    total_time = 0

    ##############################
    ##Drone initial orientation###
    ##############################
    facing_human = False
    while not facing_human:

        #CV
        img1 = drone1.get_frame_read()
        if img1 is not None:
            logging.debug("Processed frame")
            turn_1 = drone_1_CV.center_subject(img1, 1)

            #turn
            if turn_1 == 0: #if no human detected, continue turning
                turn_1 = 25
            elif turn_1 == 1: #human centered
                facing_human = True
                turn_1 = 0
            drone1.send_rc(0, 0, 0, turn_1)
            time.sleep(0.3)
        else:
            logging.debug("No frame recieved")

    print("\n\n\nnow moving paths\n\n\n")
    ##############################
    #########Path Planning########
    ##############################
    drone_1_pos[2] = -1 * drone1.get_yaw()

    while total_time < 60:
        #update total time
        total_time = time.time() - start_time

        #CV
        
        img1 = drone1.get_frame_read()
        if img1 is not None:
            logging.debug("Processed frame")
            turn_1 = drone_1_CV.center_subject(img1, 1)
            if turn_1 == 1:
                turn_1 = 0
            
            #update timer
            if iter == 0:
                sleep_time = 0 #do not update positions for the first loop
                iter += 1
            else:
                sleep_time = time.time() - timer
            
            if not drone_1_terminate:
                #calculate new angle
                print(f"updating position: {drone_1_pos}")
                drone_1_pos[2] = -1 * drone1.get_yaw()

                #calculate new position
                theta_x_component_change = 0
                if drone_1_movement[0] > 0:
                    theta_x_component_change = -1
                else:
                    theta_x_component_change = 1
                theta_x_component = (drone_1_pos[2] + 90 * theta_x_component_change) % 360
                theta_y_component = (drone_1_pos[2]) % 360
                delta_x = abs(drone_1_movement[0]) * math.cos(math.radians(theta_x_component)) + drone_1_movement[1] * math.cos(math.radians(theta_y_component))
                delta_y = abs(drone_1_movement[0]) * math.sin(math.radians(theta_x_component)) + drone_1_movement[1] * math.sin(math.radians(theta_y_component))
                drone_1_pos[0] += delta_x * sleep_time
                drone_1_pos[1] += delta_y * sleep_time

            else:
                go_up = 0
                drone_1_pos[2] = -1 * drone1.get_yaw()

            #path planning
            drone_1_movement = drone_1_path_plan.move_towards_goal(drone_1_pos[0], drone_1_pos[1], drone_1_pos[2], drone_1_terminate)
            if drone_1_movement[0] == 0.1:
                drone_1_terminate = True
                print("\n\n\nnow stopping drone movement\n\n\n")
                drone_1_movement[0], drone_1_movement[1] = 0, 0
            
            #move drone and update values, already considered if drone terminated
            drone1.send_rc(drone_1_movement[0], drone_1_movement[1], go_up, turn_1)

            timer = time.time() #time for keeping track of how much to update drones positions

        else: 
            logging.debug("no frame recieved")

    #clean up
    time.sleep(5)
    cv2.destroyAllWindows()
    drone1.land()
    
    drone1.streamoff()
    drone1.stop_drone_video()




except KeyboardInterrupt:
    logging.info("KeyboardInterrupt received. Landing the drones...")
    drone1.land()
    
    drone1.streamoff()
    drone1.stop_drone_video()
    #drone1.end()
    #drone2.end()