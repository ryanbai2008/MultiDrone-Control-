import socket
import threading
import cv2
import customtello
# Tello IP and port configuration
TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
TELLO_ADDRESS = (TELLO_IP, TELLO_PORT)
LOCAL_IP = '192.168.10.2'  # Updated local IP
LOCAL_PORT = 0
LOCAL_ADDRESS = (LOCAL_IP, LOCAL_PORT)
drone = customtello.myTello(LOCAL_IP)
drone.connect()
drone.streamon()
drone.start_video_thread()