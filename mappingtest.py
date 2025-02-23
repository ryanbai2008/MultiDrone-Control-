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
from customtello import myTello
import path_planner
import tello_tracking
import collision
import logging

def updateScreen():
    startingyaw1 = 0
    startingyaw2 = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False    
       
        startMap.start_screen(0, 0, 0, 0, 0, 0, 0, 0)
        sleep(10)

    pygame.quit()
    sys.exit()

#creates a map of the environment on pygame
pygame.init()
screen = pygame.display.set_mode([864, 586])
screen_width, screen_height = pygame.display.get_surface().get_size()
pygame.display.set_caption("Path Planning with Map (BRViz)")
screen.fill((255, 255, 255))

isRunning = True
sizeCoeff = 400/42 # actual distance/pixel distance in cm (CHANGE THIS VALUE IF YOUR CHANGING THE MAP)

def scaleImgDown(img, scale_factor):
    original_width, original_height = img.get_size()
    new_width = int(original_width * scale_factor)
    new_height = int(original_height * scale_factor)
    img = pygame.transform.smoothscale(img, (new_width, new_height))# Scale the image
    return img

startMap = map.initializeMap(screen, "Make Drone 1 Path")
startMap.start_screen(0, 0, 0, 0, 0, 0, 0, 0)

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
startMap.start_screen(0, 0, 0, 0, 0, 0, 0, 0)

#DRONE 2 Mapping
map2 = map.mapStart(sizeCoeff, screen, Background('pathPlanned.png', [0, 105], 1))
angle2, distanceInCm2, distanceInPx2, path2 = map2.createMap()
startMap.changeInstruction("Add the Subject")
pygame.draw.line(screen, (0, 0, 0), path2[1], path2[2], 6) #creates a line as the edges                
pygame.draw.circle(screen, (0, 0, 255), path2[1], 5) #To note where the nodes are
pygame.draw.circle(screen, (0, 0, 255), path2[2], 5) #To note where the nodes are

startMap.start_screen(0, 0, 0, 0, 0, 0, 0, 0)
pygame.display.update()

personx, persony, personpospx = map2.addPerson(sizeCoeff)
personpos = (personx, persony)

startMap.changeInstruction("Moving Drones...")
startMap.start_screen(0, 0, 0, 0, 0, 0, 0, 0)

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
    #collisiondetect = collision(path2[0], intersection, 500, 500)
    #collisiondetect.get_vertex() 
    intersection = True
print(intersection)
print(personpos)

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
angularSpeed = 50

timeDur = distanceInCm/linearSpeed
rotationDur = angle/angularSpeed

timeDur2 = distanceInCm2/linearSpeed
rotationDur2 = angle2/angularSpeed

drone1num_steps = int(timeDur / updateTime)
drone1angle_num_steps = int(rotationDur / angleUpdateTime)

drone2current_pos = list(start_pos2)
drone2previous_pos= drone2current_pos.copy()
drone2num_steps = int(timeDur2 / updateTime)
drone2angle_num_steps = int(rotationDur2 / angleUpdateTime)


# Calculate the increments in x and y directions
dx1 = (end_pos1[0] - start_pos1[0]) / drone1num_steps
dy1 = (end_pos1[1] - start_pos1[1]) / drone1num_steps

dx2 = (end_pos2[0] - start_pos2[0]) / drone2num_steps
dy2 = (end_pos2[1] - start_pos2[1]) / drone2num_steps

initial_yaw = 0  # Initial yaw angle in degrees
target_yaw1 = map1.get_angle(path[0], personpospx, (path[0][0]+10, path[0][1]))
target_yaw2 = map2.get_angle(path2[0], personpospx, (path2[0][0]+10, path2[0][1]))
print(target_yaw1)
print(target_yaw2)

drone1points.append(drone1current_pos)
drone2points.append(drone2current_pos)

#start_time = pygame.time.get_ticks()  # Get the start time
#rotation_done = False
drawPoints(screen, drone1points, drone1Img, yaw1)
drawPoints(screen, drone2points, drone2Img, yaw2)

#updates the screen
screenThread = threading.Thread(target=updateScreen)
screenThread.start()

#delays for the takeoff time
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False    

    screen.blit(background.image, (0, 0))    

    if yaw1 != target_yaw1:
        print(yaw1)
        yaw1 += (abs(target_yaw1) - initial_yaw) / drone1angle_num_steps
        if abs(yaw1 - target_yaw1) < 0.1:
            yaw1 = target_yaw1  # Snap to target yaw if close
    else:
        if step1 <= drone1num_steps:
            previous_pos1 = drone1current_pos.copy()
            drone1current_pos[0] += dx1
            drone1current_pos[1] += dy1
            drone1points.append(tuple(drone1current_pos))
            # Increment yaw angle
            drone1points.append((int(drone1current_pos[0]), int(drone1current_pos[1])))
            step1 += 1


    if yaw2 != target_yaw2:
        yaw2 += (abs(target_yaw2) - initial_yaw) / drone2angle_num_steps
        if abs(yaw2 - target_yaw2) < 0.1:
            yaw2 = target_yaw2  # Snap to target yaw if close
    else:
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

