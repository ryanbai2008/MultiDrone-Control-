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


#set up logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

#brayden and ryan custom API for drones
class myTello:
    def __init__(self, wifi_adapter_ip, video_port):
        self.TELLO_IP = "192.168.10.1"
        self.PORT = 8889
        self.BUFFER_SIZE = 4096
        self.VIDEO_PORT = 11111
        self.wifi_adapter_ip = wifi_adapter_ip
        self.sock = None
        self.frame = None
        self.frame_lock = Lock()
        self.video_thread = None
        self.running = False
        self.stop_video = False
        self.VIDEO_PORT = video_port
        self.connected = False

    # Initialize and bind the UDP socket
    def init_socket(self):
        if not self.sock:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.sock.bind((self.wifi_adapter_ip, 0))
            except Exception as e:
                logging.error(f"Bind failed: {e}")
                exit()
            self.host, self.port = self.sock.getsockname()
            logging.info(f"Socket bound to {self.host}:{self.port}")
        else:
            logging.info("Socket already initialized")

    # Function to send command to Tello drone
    def send_command(self, command, retries=5):
        for attempt in range(retries):
            try:
                self.sock.sendto(command.encode('utf-8'), (self.TELLO_IP, self.PORT))
                logging.info(f"Sent command to drone via {self.wifi_adapter_ip}: {command}")
                self.sock.settimeout(2.0)  # Set a timeout for the response
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
        while self.connected:
            self.getBattery()
            time.sleep(interval)

    def takeoff(self):
        self.send_command("takeoff")

    def streamon(self):
        self.send_command("streamon")

    def streamoff(self):
        self.send_command("streamoff")

    # Function to decode and display the video stream
    def receive_video(self, droneid):
        video_stream_url = f'udp://@{self.wifi_adapter_ip}:{self.VIDEO_PORT}'
        logging.info(f"Starting video stream for drone {droneid} at {self.TELLO_IP}:{self.VIDEO_PORT}")

        video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        video_sock.bind((self.wifi_adapter_ip, self.VIDEO_PORT))  # Bind the local video port
    
            
            # Initialize the video capture object with the URL of the drone's video feed
        while self.running:
            if self.stop_video:  # Check if the video stream should stop
                break
           #cap = cv2.VideoCapture(video_stream_url)       
            data, _ = video_sock.recvfrom(self.BUFFER_SIZE)
            frame = np.frombuffer(data, dtype=np.uint8)  # Convert received data to a NumPy array
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)  # Decode image
            #ret, frame = cap.read()
            if frame is not None:
                frame = cv2.resize(frame, (960, 720))
                with self.frame_lock:
                    self.frame = frame
                #cv2.imshow(f"Drone {droneid}", self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        video_sock.close()
        cv2.destroyAllWindows()

    def get_frame_read(self):
        with self.frame_lock:
            return self.frame
        

        
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
            logging.info("Stopped video stream")
    
    def emergency(self):
        self.send_command("emergency")

    #automatically turns stream on
    def start_video_thread(self, droneid):
        self.running = True
        self.video_thread = Thread(target=self.receive_video, args=(droneid,))
        self.video_thread.start()

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

