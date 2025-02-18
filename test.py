import socket
import threading
import cv2
import customtello
import time
import tello_tracking
# Tello IP and port configuration
TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
TELLO_ADDRESS = (TELLO_IP, TELLO_PORT)
LOCAL_IP = '192.168.10.3'  # Updated local IP
LOCAL_PORT = 0
LOCAL_ADDRESS = (LOCAL_IP, LOCAL_PORT)
drone = customtello.myTello(LOCAL_IP)
drone_CV = tello_tracking.CV()

drone.connect()
drone.streamon()
drone.start_video_thread()
drone.get_frame_read()
while not human_yes_1:
    img1 = drone.get_frame_read()
    drone_CV.center_subject(img1)   
    turn_left_1, human_yes_1 = drone_CV.center_subject(img1)
    print(human_yes_1)
