import pygame
import json
import math
import cv2 
import time
from time import sleep
import numpy as np
import sys
from mapBackground import Background
import map
from localizeIRT import localizer
import socket
import os
#from customtello import VideoProxyServer
from customtello import myTello

import path_planner
import tello_tracking_2 as tello_tracking
import collision
import logging
import platform
import subprocess
import avoid
import re


# Define the IP addresses of the two Wi-Fi adapters
WIFI_ADAPTER_1_IP = "192.168.10.2"  # IP address of Wi-Fi Adapter 1 (connected to Drone 1)
WIFI_ADAPTER_2_IP = "192.168.10.3"  # IP address of Wi-Fi Adapter 2 (connected to Drone 2)

drone1 = myTello(WIFI_ADAPTER_1_IP, 11111)
drone2 = myTello(WIFI_ADAPTER_2_IP, 11111)

drone1.connect()
drone2.connect()

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
sizeCoeff = 531.3/57 # actual distance/pixel distance in cm (CHANGE THIS VALUE IF YOUR CHANGING THE MAP)

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

#has goal been reached for drones
drone_1_terminate = False
drone_2_terminate = False

#how frequently the position is updated
sleep_time = 0
timer = 0
iter = 0

pygame.quit()

normal_height = 200

#turn on drones cameras
drone1.streamon()
drone2.streamon()
drone1.start_video_stream(1)
drone2.start_video_stream(2)
time.sleep(2)
drone1.takeoff()
drone2.takeoff()

#path
start_1_X, start_1_Y, end_1_X, end_1_Y = path[0][0], path[0][1], path[1][0], path[1][1]
path1 = [start_1_X, start_1_Y, end_1_X, end_1_Y]

#other drone path
path_2 = [path2[0][0],path2[0][1], path2[1][0], path2[1][1]]

#drone current values
drone_1_pos = [path1[0], path1[1], 0] #(X, Y, angle), STARTING ANGLE MUST BE 0 DEGREES
drone_2_pos = [path_2[0], path_2[1], 0]

#drone movement
drone_1_movement = [0, 0, 0] #(delta X, delta Y, delta angle)

#path planning and CV and collision objects
drone_1_path_plan = path_planner.PathPlan(path1[0], path1[2], path1[1], path1[3], drone_1_pos[2])
drone_2_path_plan = path_planner.PathPlan(path_2[0], path_2[2], path_2[1], path_2[3], drone_2_pos[2])
drone_1_CV = tello_tracking.CV(1)
drone_2_CV = tello_tracking.CV(2)

drone_collision = avoid.Avoid(path1, path_2)

#goal reached for drones?
drone_1_terminate = False
drone_2_terminate = False

# #turn on drone

# drone1.send_rc(0, 0, 40, 0)
# drone2.send_rc(0, 0, 40, 0)
# time.sleep(1.5)
# drone_height = 60
# normal_height = 60
# go_up = 0
# drone1.send_rc(0, 0, 0, 0)
# drone2.send_rc(0, 0, 0, 0)

#timer for position updates
sleep_time = 0
timer = 0
iter = 0
iter_2 = 0

#total time elapsed
start_time = time.time()
total_time = 0

##############################
##Drone initial orientation###
##############################
facing_human_1 = False
facing_human_2 = False
while (not facing_human_1) or (not facing_human_2):
    #CV
    img1 = drone1.get_frame_read()
    img2 = drone2.get_frame_read()

    if img1 is not None:
        logging.debug("Processed frame for drone 1")
        turn_1 = drone_1_CV.center_subject(img1, 1)

        #turn 1
        if turn_1 == 0: #if no human detected, continue turning
            turn_1 = 25
        elif turn_1 == 1: #human centered
            facing_human_1 = True
            turn_1 = 0
        drone1.send_rc(0, 0, 0, turn_1)
    else:
        logging.debug("No frame recieved for drone 1")

    if img2 is not None:
        logging.debug("Processed frame for drone 2")
        turn_2 = drone_2_CV.center_subject(img2, 2)

        #turn 2
        if turn_2 == 0: #if no human detected, continue turning
            turn_2 = 25
        elif turn_2 == 1: #human centered
            facing_human_2 = True
            turn_2 = 0
        drone2.send_rc(0, 0, 0, turn_2)
    else:
        logging.debug("No frame recieved for drone 2")
    #time.sleep(0.1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    

print("\n\n\nnow moving paths\n\n\n")
##############################
#########Path Planning########
##############################
drone_1_pos[2] = -1 * drone1.get_yaw()
drone_2_pos[2] = -1 * drone2.get_yaw()

while total_time < 60:
    #update total time
    total_time = time.time() - start_time

    #CV
    img1 = drone1.get_frame_read()
    img2 = drone2.get_frame_read()

    if img1 is not None:
        logging.debug("Processed frame for drone 1")
        turn_1 = drone_1_CV.center_subject(img1, 1)
        if turn_1 == 1:
            turn_1 = 0 #if subject centered, no turn
        #update timer
        if iter == 0:
            sleep_time = 0 #do not update positions for the first loop
            iter += 1
        else:
            sleep_time = time.time() - timer
            
        if (not drone_1_terminate):
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

            #detect collision and manage heights
            drone_height += go_up * sleep_time
            print(f"drone height: {drone_height}, normal height: {normal_height}")
            collision_check = drone_collision.detect_collision(drone_1_pos[0], drone_1_pos[1])
            if collision_check == "collision":
                go_up = 40
            elif collision_check == "no collision" and drone_height > normal_height * 1.1: #buffer
                go_up = -20
            elif collision_check == "no collision" and drone_height < normal_height * 1.1: #buffer
                go_up = 20
            else:
                go_up = 0
            
            if drone_height > 250:
                drone1.land()
                drone2.land()
                break
        drone_1_movement = drone_1_path_plan.move_towards_goal(drone_1_pos[0], drone_1_pos[1], drone_1_pos[2], drone_1_terminate)
        if drone_1_movement[0] == 0.1:
                        drone_1_terminate = True
                        print("\n\n\nnow stopping drone movement drone 1\n\n\n")
                        drone_1_movement[0], drone_1_movement[1] = 0, 0
        #drone1.send_rc(drone_1_movement[0], drone_1_movement[1], go_up, turn_1)
        print(f"Drone 1 Position: {drone_1_pos}, Movement: {drone_1_movement}")
        #time.sleep(0.1)
        timer = time.time() #time for keeping track of how much to update drones positions

    if img2 is not None:
        logging.debug("Processed frame for drone 2")
        turn_2 = drone_2_CV.center_subject(img2, 2)

        if turn_2 == 1:
            turn_2 = 0 #if subject centered, no turn
            #update timer
        if iter_2 == 0:
            sleep_time = 0 #do not update positions for the first loop
            iter_2 += 1
        else:
            sleep_time = time.time() - timer

        if(not drone_2_terminate):
            drone_2_pos[2] = -1 * drone2.get_yaw()
            #calculate new position
            theta_x_component_change = 0
            if drone_2_movement[0] > 0:
                theta_x_component_change = -1
            else:
                theta_x_component_change = 1
            theta_x_component = (drone_2_pos[2] + 90 * theta_x_component_change) % 360
            theta_y_component = (drone_2_pos[2]) % 360
            delta_x = abs(drone_2_movement[0]) * math.cos(math.radians(theta_x_component)) + drone_2_movement[1] * math.cos(math.radians(theta_y_component))
            delta_y = abs(drone_2_movement[0]) * math.sin(math.radians(theta_x_component)) + drone_2_movement[1] * math.sin(math.radians(theta_y_component))
            drone_2_pos[0] += delta_x * sleep_time
            drone_2_pos[1] += delta_y * sleep_time

        #path planning
        drone_2_movement = drone_2_path_plan.move_towards_goal(drone_2_pos[0], drone_2_pos[1], drone_2_pos[2], drone_2_terminate)

        #reached goal?
        
        if drone_2_movement[0] == 0.1:
            drone_2_terminate = True
            print("\n\n\nnow stopping drone movement drone 2\n\n\n")
            drone_2_movement[0], drone_2_movement[1] = 0, 0
        
        #move drone and update values, already considered if drone terminated
        #drone2.send_rc(drone_2_movement[0], drone_2_movement[1], 0, turn_2)
        print(f"Drone 2 Position: {drone_2_pos}, Movement: {drone_2_movement}")
        #time.sleep(0.1)

        timer = time.time() #time for keeping track of how much to update drones positions

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


#clean up
time.sleep(5)
cv2.destroyAllWindows()
drone1.land()
drone2.land()
drone1.streamoff()
drone2.streamoff()


