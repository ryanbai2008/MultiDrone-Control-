import socket
import time
from threading import Thread, Lock
import cv2
import numpy as np
import logging
import re
import threading
import os
import platform
import subprocess
from collections import deque


#set up logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

#brayden and ryan custom API for drones
class myTello:
    def __init__(self, wifi_adapter_ip, video_port, buffer_size=5):
        self.TELLO_IP = "192.168.10.1"
        self.PORT = 8889
        self.BUFFER_SIZE = 65536  # 64 KB
        self.VIDEO_PORT = video_port
        self.wifi_adapter_ip = wifi_adapter_ip
        self.sock = None
        self.frame = None
        self.frame_lock = Lock()
        self.video_thread = None
        self.running = False
        self.stop_video = False
        self.VIDEO_PORT = video_port
        self.connected = False
        self.frame_buffer = deque(maxlen=buffer_size)  # Frame buffer to store frames
        self.isOn = True


    # Initialize and bind the UDP socket
    def init_socket(self):
        if not self.sock:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.sock.bind((self.wifi_adapter_ip, self.VIDEO_PORT))
            except Exception as e:
                logging.error(f"Bind failed: {e}")
                exit()
            self.host, self.port = self.sock.getsockname()
            logging.info(f"Socket bound to {self.host}:{self.port}")
        else:
            logging.info("Socket already initialized")

    # Function to send command to Tello drone
    def send_command(self, command, retries=5):
        if self.sock is None or self.sock.fileno() == -1:
            logging.error("Socket not initialized. Cannot send command.")
            self.init_socket()
        for attempt in range(retries):
            try:
                self.sock.sendto(command.encode('utf-8'), (self.TELLO_IP, self.PORT))
                logging.info(f"Sent command to drone via {self.wifi_adapter_ip}: {command}")
                self.sock.settimeout(5.0)  # Set a timeout for the response
                response, _ = self.sock.recvfrom(self.BUFFER_SIZE)
                response_decoded = response.decode('utf8') #gets the respond
                logging.debug(f"Response: {response_decoded}")
                return response_decoded
            except socket.timeout:
                logging.error(f"Timeout waiting for response to command '{command}'")
            except Exception as e:
                logging.error(f"Error sending command '{command}': {e}")
        logging.error(f"Failed to send command '{command}' after {retries} attempts")
        return None
    
    def get_command(self, command, retries=5):
        for attempt in range(retries):
            try:
                self.sock.sendto(command.encode('utf-8'), (self.TELLO_IP, self.PORT))
                logging.info(f"Sent command to drone via {self.wifi_adapter_ip}: {command}") #message to send command
                response, _ = self.sock.recvfrom(self.BUFFER_SIZE)
                return response.decode('utf-8') #gets the response
            except Exception as e:
                logging.error(f"Error getting command '{command}': {e}")
        logging.error(f"Failed to get command '{command}' after {retries} attempts")
        return None
    
    def connect(self):
        if self.sock is None or self.sock.fileno() == -1:
            self.init_socket()
        # Connect to the drones by sending the 'command' mode
        if not self.connected:
            response = self.send_command("command")
            if response and "ok" in response.lower():
                self.connected = True
                logging.info("Connected to drone")
            else:
                logging.error("Failed to connect to drone. Retrying...")
                self.connected = False

    def keep_alive(self, interval=10):
        """Keeps the drone connection alive by sending 'battery?' command every few seconds."""
        while self.isOn:
            self.getBattery()
            time.sleep(interval)
    
    def off(self):
        self.isOn = False

    def takeoff(self):
        self.send_command("takeoff")

    def streamon(self):
        self.send_command("streamon")

    def streamoff(self):
        self.send_command("streamoff")

    # Function to decode and display the video stream
    def receive_video(self, droneid):
        
        video_stream_url = f'udp://@0.0.0.0:11111'
        logging.info(f"Starting video stream for drone {droneid} at {self.TELLO_IP}:{self.VIDEO_PORT}")

            
            # Initialize the video capture object with the URL of the drone's video feed
        cap = cv2.VideoCapture(video_stream_url)      
        if not cap.isOpened():
            logging.error(f"Failed to open video stream for drone {droneid}")
            return


        while self.running:
            if self.stop_video:  # Check if the video stream should stop
                break
            if ret:
                ret, frame = cap.read()

            
                with self.frame_lock:
                    self.frame_buffer.append(frame)  # Add frame to the buffer
                    self.frame = frame

                    cv2.imshow(f"Drone {droneid}", self.frame)
            else: # If the frame is not read, stop the video stream
                logging.error(f"Failed to read frame from drone {droneid}")
                continue
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

    def get_frame_read(self):
       with self.frame_lock:
           return self.frame
    # def get_frame_read(self):
    #     # Try acquiring the lock without blocking
    #     if self.frame_lock.acquire(blocking=False):  # Non-blocking lock acquisition
    #         try:
    #             if len(self.frame_buffer) > 0:
    #                 return self.frame_buffer[-1]  # Return the most recent frame
    #             return self.frame  # Return the current frame if buffer is empty
    #         finally:
    #             self.frame_lock.release()
    #     else:
    #         return None  # Return None if the lock was not acquired

        
    def stop_video_stream(self):
        self.stop_video = True
        self.running = False

    def stop_drone_video(self):
        if self.running:
            self.running = False
            self.stop_video_stream()  # stop video stream 
            if self.video_thread is not None:
                self.video_thread.join()
            self.frame = None
            self.frame_lock = Lock()
            self.video_thread = None
            self.frame_buffer.clear()  # Clear the frame buffer

            logging.info("Stopped video stream")
    
    def emergency(self):
        self.send_command("emergency")

    #automatically turns stream on
    def start_video_thread(self, droneid):
        if not self.running:
            self.running = True
            self.video_thread = Thread(target=self.receive_video, args=(droneid,))
            self.video_thread.start()
            logging.info("Started video stream")
        else:
            logging.warning(f"Video stream already running for {droneid}")

    def end(self):
        self.stop_video_stream()  # stop video stream 
        if self.video_thread is not None:
            self.running = False
            self.video_thread.join()
        if self.sock:
            self.sock.close()
            self.sock = None  #  reset the socket
        self.frame = None
        self.frame_lock = Lock()
        self.video_thread = None
        self.connected = False
        logging.info("Disconnected from drone")

    def getConnected(self):
        return self.connected


    def moveForward(self, distance):
        # Move drones forward
        self.send_command(f"forward {distance}")

    def moveBackward(self, distance):
        self.send_command(f"back {distance}")

    def moveLeft(self, distance):
        self.send_command(f"left {distance}")

    def moveRight(self, distance):
        self.send_command(f"right {distance}")

    def move_up(self, distance):
        self.send_command(f"up {distance}")

    def move_down(self, distance):
        self.send_command(f"down {distance}")

    def send_rc(self, roll, pitch, throttle, yaw):
        self.send_command(f"rc {roll} {pitch} {throttle} {yaw}")

    def curve(self, x1, y1, z1, x2, y2, z2, speed):
        self.send_command(f"curve {x1} {y1} {z1} {x2} {y2} {z2} {speed}")

    def go(self, x, y, z, speed):
        self.send_command(f"go {x} {y} {z} {speed}")

    def land(self):
        # Land both drones
        self.send_command("land")

    def rotateCCW(self, angle):
        self.send_command(f'ccw {abs(angle)}')

    def rotateCW(self, angle):
        self.send_command(f'cw {abs(angle)}')

    def get_yaw(self):
        imudata = self.get_command("attitude?")
        print(imudata)
        match = re.search(r'yaw:\s*(-?\d+\.?\d*)', imudata)

        if match:
            # Extract and return the yaw value from the matched group
            current_yaw = float(match.group(1))  # The captured yaw value
            logging.info(f"Current Yaw: {current_yaw}")
            return current_yaw
       
        # Calculate the change in yaw

    def getBattery(self):
        battery = self.get_command("battery?")

        try:
            match = re.search(r'\d+', battery)
            if match:
                return int(match.group())
            else:
                logging.error(f"Could not parse battery from response: {battery}")
                return None
        except Exception as e:
            logging.error(f"Error parsing battery: {e}")
            return None


    def getHeight(self):
        height = self.get_command("height?")

        try:
            match = re.search(r'\d+', height)
            if match:
                return int(match.group())
            else:
                logging.error(f"Could not parse height from response: {height}")
                return None
        except Exception as e:
            logging.error(f"Error parsing height: {e}")
            return None
    
    def get_speed(self):
        speed_data = self.get_command("speed?")
        try:
            match = re.search(r'\d+', speed_data)
            if match:
                return int(match.group())
            else:
                logging.error(f"Could not parse speed from response: {speed_data}")
                return None
        except Exception as e:
            logging.error(f"Error parsing speed: {e}")
            return None
        
    def get_AngularSpeed(self, startingyaw):
        currentyaw = 0

        # Calculate the change in yaw
        yaw_difference = currentyaw - startingyaw

        # Handle potential yaw wraparound (like from 360° back to 0°)
        if yaw_difference > 180:
            yaw_difference -= 360
        elif yaw_difference < -180:
            yaw_difference += 360
        angular_speed1 = yaw_difference / 1  
        
        return angular_speed1

# class VideoProxyServer:
#     def __init__(self, drone_ips, server_ip, base_port):
#         self.drone_ips = drone_ips
#         self.server_ip = server_ip
#         self.base_port = base_port

#     def start_proxy(self):
#         for i, drone_ip in enumerate(self.drone_ips):
#             drone_port = 11111  # Tello video port
#             local_port = self.base_port + i
#             start = threading.Thread(target=self.proxy_video, args=(drone_ip, drone_port, local_port))
#             start.start()

#     def proxy_video(self, drone_ip, drone_port, local_port):
#         sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         sock.bind((self.server_ip, local_port))
#         drone_addr = (drone_ip, drone_port)

#         while True:
#             data, _ = sock.recvfrom(4096)
#             sock.sendto(data, drone_addr)

