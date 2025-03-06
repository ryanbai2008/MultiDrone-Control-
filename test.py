import socket
import threading
import cv2
import customtello
import time
import tello_tracking
import logging

TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
TELLO_ADDRESS = (TELLO_IP, TELLO_PORT)
LOCAL_IP = '192.168.10.2'  
LOCAL_IP2 = '192.168.10.4' 

LOCAL_PORT = 0
LOCAL_ADDRESS = (LOCAL_IP, LOCAL_PORT)
drone = customtello.myTello(LOCAL_IP, 11111)
#drone2 = customtello.myTello(LOCAL_IP2), 11111

drone1_CV = tello_tracking.CV()
#drone2_CV = tello_tracking.CV()

drone.connect()
#drone2.connect()
drone.streamon()
#drone2.streamon()

# drone.start_video_thread(1)
# #drone2.start_video_thread()

# print(drone.get_yaw())
# print(drone.get_speed())
# human_yes_1 = False
# human_yes_2 = False

# try:
#     while not human_yes_1 and not human_yes_2:
#         # img1 = drone.get_frame_read()
#         #img2 = drone2.get_frame_read()

#         # if img1 is not None:
#         #     turn1 = drone1_CV.center_subject(img1, 1)
#         #     logging.debug("Processed frame")
#         # else:
#         #     logging.debug("No frame received")

#         # if img2 is not None:
#         #     turn2 = drone1_CV.center_subject(img2, 1)
#         #     logging.debug("Processed frame")
#         # else:
#         #     logging.debug("No frame received")

# except KeyboardInterrupt:
#     print("Exiting program...")
# finally:
#     drone.stop_video_stream()  
#     drone.end()  
#     print("Program exited cleanly.")