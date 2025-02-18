import socket
import threading
import cv2
import customtello
import time
import tello_tracking
import logging

# Tello IP and port configuration
TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
TELLO_ADDRESS = (TELLO_IP, TELLO_PORT)
LOCAL_IP = '192.168.10.2'  # Updated local IP
LOCAL_PORT = 0
LOCAL_ADDRESS = (LOCAL_IP, LOCAL_PORT)
drone = customtello.myTello(LOCAL_IP)
drone_CV = tello_tracking.CV()

drone.connect()
drone.streamon()
drone.start_video_thread()
print(drone.get_yaw())
print(drone.get_speed())

human_yes_1 = False
while not human_yes_1:
    img1 = drone.get_frame_read()
    if img1 is not None:
        turn1 = drone_CV.center_subject(img1, 1)  
        logging.debug("Processed frame")
    else:
        logging.debug("no frame recieved")

        