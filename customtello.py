import socket
import time
from threading import Thread, Lock
import cv2
import numpy as np
import logging
import re

#set up logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

#brayden and ryan custom API for drones
class myTello:
    def __init__(self, wifi_adapter_ip):
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

        # Initialize and bind the UDP socket
    def init_socket(self):
        if self.sock:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.wifi_adapter_ip, 0))
        except Exception as e:
            logging.error(f"Bind failed: {e}")
            exit()
        # Get the port that was automatically selected
        self.host, self.port = self.sock.getsockname()
        logging.info(f"Socket bound to {self.host}:{self.port}")

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
        self.init_socket()  # Initialize and bind socket
        # Connect to the drones by sending the 'command' mode
        self.send_command("command")

    def takeoff(self):
        self.send_command("takeoff")

    def streamon(self):
        self.send_command("streamon")

    def streamoff(self):
        self.send_command("streamoff")

    # Function to decode and display the video stream
    def receive_video(self):
        cap = cv2.VideoCapture('udp://@0.0.0.0:11111')

        while True:
            if self.stop_video:  # Check if the video stream should stop
                break

            ret, frame = cap.read()
            if ret:
                with self.frame_lock:
                    self.frame = frame
                #cv2.imshow(self.wifi_adapter_ip, self.frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def get_frame_read(self):
        with self.frame_lock:
            return self.frame
        
    def stop_video_stream(self):
        self.stop_video = True
    
    def emergency(self):
        self.send_command("emergency")

    #automatically turns stream on
    def start_video_thread(self):
        self.running = True
        self.video_thread = Thread(target=self.receive_video)
        self.video_thread.start()

    def end(self):
        self.running = False
        self.stop_video_stream()  # Stop the video stream before ending
        if self.video_thread is not None:
            self.video_thread.join()
        self.sock.close()
        time.sleep(2)
        self.connect()

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
        self.sock.close()
        time.sleep(2)
        self.connect()

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
        return battery


    def getHeight(self):
        height = self.get_command("height?")
        return height
    
    def get_speed(self):
        speed_data = self.get_command("speed?")
        return speed_data

    def get_AngularSpeed(self, startingyaw):
        currentyaw = self.get_yaw()

        # Calculate the change in yaw
        yaw_difference = currentyaw - startingyaw

        # Handle potential yaw wraparound (like from 360° back to 0°)
        if yaw_difference > 180:
            yaw_difference -= 360
        elif yaw_difference < -180:
            yaw_difference += 360
        angular_speed1 = yaw_difference / 1  
        
        return angular_speed1
