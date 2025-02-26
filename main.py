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


# Define the IP addresses of the two Wi-Fi adapters
WIFI_ADAPTER_1_IP = "192.168.10.2"  # IP address of Wi-Fi Adapter 1 (connected to Drone 1)
WIFI_ADAPTER_2_IP = "192.168.10.3"  # IP address of Wi-Fi Adapter 2 (connected to Drone 2)

drone_ips = [WIFI_ADAPTER_1_IP, WIFI_ADAPTER_2_IP]
server_ip = "192.168.209.193"  # Replace with your actual server IP
base_port = 30000
TELLO_IP = "192.168.10.1"

def run_command(command):
    """Runs a system command and returns output."""
    try:
        subprocess.run(command, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running command: {e}")

def connect_wifi(adapter, ssid):
    """Connects Wi-Fi adapter to a specific Tello network."""
    system = platform.system()
    
    if system == "Windows":
        run_command(f'netsh wlan connect name="{ssid}" interface="{adapter}"')
    else:  # Linux/macOS
        run_command(f'nmcli device wifi connect "{ssid}" ifname {adapter}')
    
    time.sleep(5)

def set_static_ip(adapter, ip):
    """Assigns a static IP to the Wi-Fi adapter."""
    system = platform.system()
    
    if system == "Windows":
        run_command(f'netsh interface ip set address name="{adapter}" static {ip} 255.255.255.0')
    else:  # Linux/macOS
        run_command(f'sudo ifconfig {adapter} {ip} netmask 255.255.255.0 up')

def add_route():
    """Adds routing rules to direct traffic to the correct adapter."""
    system = platform.system()
    
    if system == "Windows":
        run_command(f'route -p add {TELLO_IP} mask 255.255.255.255 {WIFI_ADAPTER_1_IP} metric 1')
        run_command(f'route -p add {TELLO_IP} mask 255.255.255.255 {WIFI_ADAPTER_2_IP} metric 1')
    else:  # Linux/macOS
        run_command(f'sudo ip route add {TELLO_IP} via {WIFI_ADAPTER_1_IP} dev wlan0')
        run_command(f'sudo ip route add {TELLO_IP} via {WIFI_ADAPTER_2_IP} dev wlan1')

WIFI_1 = "Wi-Fi"
WIFI_2 = "Wi-Fi 2"

connect_wifi(WIFI_1, "TELLO-D06F9F")
connect_wifi(WIFI_2, "TELLO-EE4263")

#debug
#netsh wlan show interfaces   
#route print                

# Assign static IPs
set_static_ip(WIFI_1, WIFI_ADAPTER_1_IP)
set_static_ip(WIFI_2, WIFI_ADAPTER_2_IP)

# Add routing rules
add_route()

#proxy_server = VideoProxyServer(drone_ips, server_ip, base_port)
#proxy_server.start_proxy()

drone1 = myTello(WIFI_ADAPTER_1_IP, base_port)
drone2 = myTello(WIFI_ADAPTER_2_IP, base_port + 1)

drone1.connect()
drone2.connect()

def move_tello(distance1, distance2, angle1, angle2):# Define the Tello IP and port
    # Take off both drones
    drone1.takeoff
    time.sleep(5)

    #rotates the first drone
    if angle1 < 0:
        print("Rotating")
        drone1.rotateCCW(abs(angle))
    elif angle1 > 0:
        print("Rotating")
        drone1.rotateCW(abs(angle))
    else:
        print("No rotation needed")

    #rotates the second drone
    if angle2 < 0:
        print("Rotating")
        drone2.rotateCCW(abs(angle2))
    elif angle2 > 0:
        print("Rotating")
        drone2.rotateCCW(abs(angle2))
    else:
        print("No rotation needed")
    time.sleep(10)

    # Move drones forward
    drone1.moveForward(distance1)
    drone2.moveForward(distance2)
  

    time.sleep(10)

    # Land both drones
    drone1.land()
    drone2.land()

battery1 = drone1.getBattery()
battery2 = drone2.getBattery()

height1 = drone1.getHeight()
height2 = drone2.getHeight()

print(battery1)
print(battery2)

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
        speedz2 =drone2.get_AngularSpeed(startingyaw2)

        battery1 = drone1.getBattery()
        battery2 = drone2.getBattery()

        height1 = drone1.getHeight()
        height2 = drone2.getHeight()

        startMap.start_screen(battery1, speedx1, speedz1, height1, battery2, speedx2, speedz2, height2)
        sleep(10)

#creates a map of the environment on pygame
pygame.init()
screen = pygame.display.set_mode([864, 586])
screen_width, screen_height = pygame.display.get_surface().get_size()
pygame.display.set_caption("Path Planning with Map (BRViz)")
screen.fill((255, 255, 255))

speedx1 = drone1.get_speed()
speedx2 = drone2.get_speed()
speedz1 = drone1.get_AngularSpeed(0)
speedz2 = drone2.get_AngularSpeed(0)
battery1 = drone1.getBattery()
battery2 = drone2.getBattery()
height1 = drone1.getHeight()
height2 = drone2.getHeight()

isRunning = True
sizeCoeff = 400/42 # actual distance/pixel distance in cm (CHANGE THIS VALUE IF YOUR CHANGING THE MAP)

def scaleImgDown(img, scale_factor):
    original_width, original_height = img.get_size()
    new_width = int(original_width * scale_factor)
    new_height = int(original_height * scale_factor)
    img = pygame.transform.smoothscale(img, (new_width, new_height))# Scale the image
    return img

startMap = map.initializeMap(screen, "Make Drone 1 Path")
startMap.start_screen(battery1, speedx1, speedz1, height1, battery2, speedx2, speedz2, height2)

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
startMap.start_screen(battery1, speedx1, speedz1, height1, battery2, speedx2, speedz2, height2)

#DRONE 2 Mapping
map2 = map.mapStart(sizeCoeff, screen, Background('pathPlanned.png', [0, 105], 1))
angle2, distanceInCm2, distanceInPx2, path2 = map2.createMap()
startMap.changeInstruction("Add the Subject")
pygame.draw.line(screen, (0, 0, 0), path2[1], path2[2], 6) #creates a line as the edges                
pygame.draw.circle(screen, (0, 0, 255), path2[1], 5) #To note where the nodes are
pygame.draw.circle(screen, (0, 0, 255), path2[2], 5) #To note where the nodes are

startMap.start_screen(battery1, speedx1, speedz1, height1, battery2, speedx2, speedz2, height2)
pygame.display.update()

personx, persony, personpospx = map2.addPerson(sizeCoeff)
personpos = (personx, persony)

print("HELLO")
startMap.changeInstruction("Moving Drones...")
startMap.start_screen(battery1, speedx1, speedz1, height1, battery2, speedx2, speedz2, height2)

print(angle)
print(distanceInCm)
print(distanceInPx)
print(path)


print(angle2)
print(distanceInCm2)
print(distanceInPx2)
print(path2)
path.pop(0)
path2.pop(0)

point1 = path[0] 
point2 = path[1] 
point3 = path2[0]
point4 = path2[1]

x1 = point1[0]
x2 = point2[0]
x3 = point3[0]
x4 = point4[0]
y1 = point1[1]
y2 = point2[1]
y3 = point3[1]
y4 = point4[1]

def line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    # Calculate the coefficients of the lines
    A1 = y2 - y1
    B1 = x1 - x2
    C1 = A1 * x1 + B1 * y1

    A2 = y4 - y3
    B2 = x3 - x4
    C2 = A2 * x3 + B2 * y3

    # Calculate the determinant of the system
    determinant = A1 * B2 - A2 * B1

    if determinant == 0:
        # Lines are parallel
        return None

    # Calculate the intersection point
    x = (B2 * C1 - B1 * C2) / determinant
    y = (A1 * C2 - A2 * C1) / determinant

    # Check if the intersection point is within the segment bounds
    if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2) and min(x3, x4) <= x <= max(x3, x4) and min(y3, y4) <= y <= max(y3, y4):
        return (x, y)
    else:
        return None
    
intersection = line_intersection(x1, y1, x2, y2, x3, y3, x4, y4)

if intersection:
    font = pygame.font.SysFont('Times',25)
    intersectx = (int)((intersection[0])/sizeCoeff)
    intersecty = (int)((screen_height  - intersection[1])/sizeCoeff)
    position_text = font.render(f'({intersectx}, {intersecty})cm', True, (128, 0, 128))
    screen.blit(position_text, intersection)
    pygame.draw.circle(screen, (128, 0, 128), intersection, 6) #purple dot at intesection 
    intersection = (intersectx, intersecty)
    collisiondetect = collision(path2[0], intersection, 500, 500)
    collisiondetect.get_vertex() 
    intersection = True
print(intersection)
print(personpos)


def move_parabolic(self, drone, speed, time, distance):
    drone2.send_rc(0, 0, 0, int(speed))
    drone.send_rc(0, 0, 0, int(speed))

#Saves the screen to be blitted
saveImg = pygame.Rect(0, 100, screen_width, screen_height-105)
# Create a new Surface to store the part of the screen
path1img = screen.subsurface(saveImg).copy()
pygame.image.save(screen, "pathPlanned2.png") #Saves new background with path


#Makes the drone image on a path
drone1Img = pygame.image.load('tello3.png')  # Replace with your image file path
drone1Img = scaleImgDown(drone1Img, 0.085) #Scale down to 8.5%

drone2Img = pygame.image.load('tello2.png')  # Replace with your image file path
drone2Img = scaleImgDown(drone2Img, 0.03) # Scale down to 3%

#position values for path planning
start_pos1X, start_pos1Y = path[0]
end_pos1X, end_pos1Y = path[1]
start_pos2X, start_pos2Y = path2[0]
end_pos2X, end_pos2Y = path2[1]

#drones current position values
drone_1_pos = [start_pos1X, start_pos1Y, 0] #initial angle of 0
drone_2_pos = [start_pos2X, start_pos2Y, 0] #initial angle of 0
drone_1_movement = [0, 0]
drone_2_movement = [0, 0]


#path planning objects and CV objects
drone_1_path_plan = path_planner.PathPlan(start_pos1X, end_pos1X, start_pos1Y, end_pos1Y, drone_1_pos[2]) #########place both drones facing to the right facing horizontally
drone_2_path_plan = path_planner.PathPlan(start_pos2X, end_pos2X, start_pos2Y, end_pos2Y, drone_2_pos[2])
drone1_CV = tello_tracking.CV()
drone2_CV = tello_tracking.CV()

#has goal been reached for drones
drone_1_terminate = False
drone_2_terminate = False


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
    start_pos2 = path2[0]
    end_pos1 = path[1]
    end_pos2 = path2[1]

    # Initialize the current position
    drone1current_pos = list(start_pos1)
    drone1previous_pos = drone1current_pos.copy()

    linearSpeed = 500
    angularSpeed = 500

    timeDur = distanceInCm/linearSpeed
    rotationDur = angle/angularSpeed

    timeDur2 = distanceInCm2/linearSpeed
    rotationDur2 = angle2/angularSpeed

    drone1num_steps = int(timeDur / updateTime)
    drone1angle_num_steps = abs(int(rotationDur / angleUpdateTime))

    drone2current_pos = list(start_pos2)
    drone2previous_pos= drone2current_pos.copy()
    drone2num_steps = int(timeDur2 / updateTime)
    drone2angle_num_steps = abs(int(rotationDur2 / angleUpdateTime))


    # Calculate the increments in x and y directions
    dx1 = (end_pos1[0] - start_pos1[0]) / drone1num_steps
    dy1 = (end_pos1[1] - start_pos1[1]) / drone1num_steps

    dx2 = (end_pos2[0] - start_pos2[0]) / drone2num_steps
    dy2 = (end_pos2[1] - start_pos2[1]) / drone2num_steps

    initial_yaw = 0  # Initial yaw angle in 
    target_yaw1 = map1.get_angle(path[0], personpospx, (path[0][0], path[0][1]+10))
    target_yaw2 = map2.get_angle(path2[0], personpospx, (path2[0][0], path2[0][1]+10))
    print(target_yaw1)
    print(target_yaw2)

    drone1points.append(drone1current_pos)
    drone2points.append(drone2current_pos)

    #start_time = pygame.time.get_ticks()  # Get the start time
    #rotation_done = False
    drawPoints(screen, drone1points, drone1Img, yaw1)
    drawPoints(screen, drone2points, drone2Img, yaw2)

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

        if rotating2:
            if yaw2 != target_yaw2:
                yaw2 += ((target_yaw2) - initial_yaw) / drone2angle_num_steps
                if abs(yaw2 - target_yaw2) < 0.1:
                    yaw2 = target_yaw2  # Snap to target yaw if close
                    rotating2 = False
        else:

            yaw2 = map1.get_angle(drone2current_pos, personpospx, (drone2current_pos[0], drone2current_pos[1]+10))

            if step2 <= drone2num_steps:
                previous_pos2 = drone2current_pos.copy()
                drone2current_pos[0] += dx2
                drone2current_pos[1] += dy2
                drone2points.append(tuple(drone2current_pos))
                # Increment yaw angle
                drone2points.append((int(drone2current_pos[0]), int(drone2current_pos[1])))
                step2 += 1
        # Draw the frame
        drawPoints(screen, drone1points, drone1Img, yaw1)

        drawPoints(screen, drone2points, drone2Img, yaw2)
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
    drone2.streamon()
    drone1.start_video_thread(1)
    drone2.start_video_thread(2)
    drone1.takeoff()
    drone2.takeoff()

    human_yes_2, human_yes_1 = False, False

    while (not human_yes_1) and (not human_yes_2):
        #CV
        img1 = drone1.get_frame_read()
        img2 = drone2.get_frame_read()
        if img1 is not None:
            turn_1 = drone1_CV.center_subject(img1, 1)
            logging.debug("Processed frame")
            if turn_1 == 1:
                human_yes_1 = True
                drone1.send_rc(0, 0, 0, 0)
            elif turn_1 == 0:
                drone1.send_rc(0, 0, 0, 20)
            else:
                drone1.send_rc(0, 0, 0, turn_1)
        else:
            logging.debug("no frame recieved")

        if img2 is not None:
            turn_2 = drone2_CV.center_subject(img2, 2)
            logging.debug("Processed frame")
            
            # if human detected to be at the center
            if turn_2 == 1:
                human_yes_2 = True
                drone2.send_rc(0, 0, 0, 0)
            elif turn_2 == 0:
                drone2.send_rc(0, 0, 0, 20)
            else:
                drone1.send_rc(0, 0, 0, turn_2)
        else:
            logging.debug("no frame recieved")
            
    while not drone_1_terminate and not drone_2_terminate:

        img1 = drone1.get_frame_read()
        img2 = drone2.get_frame_read()
        if img1 is not None:
            turn_1, __ = drone1_CV.center_subject(img1, 1)
            logging.debug("Processed frame")
        else:
            logging.debug("no frame recieved")
        if img2 is not None:
            turn_2, __ = drone2_CV.center_subject(img2, 2)
            logging.debug("Processed frame")
        else:
            logging.debug("no frame recieved")
        
        if iter == 0:
            sleep_time = 0 #do not update positions for the first loop
            iter += 1
        else:
            sleep_time = time.time() - timer
        if img1 is not None:
            if not drone_1_terminate:
                drone_1_pos[2] = drone1.get_yaw()
                theta_x_component_1 = (drone_1_pos[2] - 90 * (drone_1_pos[0] > 0)) % 360
                theta_y_component_1 = (drone_1_pos[2] + 180) % 360
                delta_x_1 = abs(drone_1_movement[0]) * math.cos(math.radians(theta_x_component_1)) + drone_1_movement[1] * math.cos(math.radians(theta_y_component_1))
                delta_y_1 = abs(drone_1_movement[0]) * math.sin(math.radians(theta_x_component_1)) + drone_1_movement[1] * math.sin(math.radians(theta_y_component_1))
                drone_1_pos[0] += delta_x_1 * sleep_time
                drone_1_pos[1] += delta_y_1 * sleep_time
            else:
                drone_1_pos[2] = drone1.get_yaw()
        if img2 is not None:
            if not drone_2_terminate:   
                drone_2_pos[2] = drone2.get_yaw()
                theta_x_component_2 = (drone_2_pos[2] - 90 * (drone_2_pos[0] > 0)) % 360
                theta_y_component_2 = (drone_2_pos[2] + 180) % 360
                delta_x_2 = abs(drone_2_movement[0]) * math.cos(math.radians(theta_x_component_2)) + drone_2_movement[1] * math.cos(math.radians(theta_y_component_2))
                delta_y_2 = abs(drone_2_movement[0]) * math.sin(math.radians(theta_x_component_2)) + drone_2_movement[1] * math.sin(math.radians(theta_y_component_2))
                drone_2_pos[0] += delta_x_2 * sleep_time
                drone_2_pos[1] += delta_y_2 * sleep_time
            else:
                drone_2_pos[2] = drone2.get_yaw()

                if intersection:
                    if(drone2.getHeight() >= drone1.getHeight() + 20):
                        drone2.send_rc(0, 0, -20, 0)
                        intersection1 = True
                        intersection = False
                        
                if intersection1:
                    if(drone2.getHeight() == drone1.getHeight()):
                        drone2.send_rc(0, 0, 0, 0)

                #path planning
                drone_1_movement = drone_1_path_plan.move_towards_goal(drone_1_pos[0], drone_1_pos[1], drone_1_pos[2], drone_1_terminate)
                drone_2_movement = drone_2_path_plan.move_towards_goal(drone_2_pos[0], drone_2_pos[1], drone_2_pos[2], drone_2_terminate)
                if drone_1_movement[0] == 0.1:
                    drone_1_terminate = True
                    drone_1_movement[0], drone_1_movement[1] = 0, 0
                if drone_2_movement[0] == 0.1:
                    drone_2_terminate = True
                    drone_2_movement[0], drone_2_movement[1] = 0, 0
                
                #move drone
                drone1.send_rc(drone_1_movement[0], drone_1_movement[1], 0, turn_1)
                drone2.send_rc(drone_2_movement[0], drone_2_movement[1], 0, turn_2)

                timer = time.time()

    drone1.land()
    drone2.land()
    drone1.streamoff()
    drone2.streamoff()
    drone1.stop_drone_video()
    drone2.stop_drone_video()
    #drone1.end()
    #drone2.end()

except KeyboardInterrupt:
    logging.info("KeyboardInterrupt received. Landing the drones...")
    drone1.land()
    drone2.land()
    drone1.streamoff()
    drone2.streamoff()
    drone1.stop_drone_video()
    drone2.stop_drone_video()
     #drone1.end()
    #drone2.end()