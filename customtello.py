import socket
import time
from threading import Thread, Lock
import cv2
import numpy as np
import logging

#set up logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

#brayden and ryan custom API for drones
class myTello:
    def __init__(self, wifi_adapter_ip):
        self.TELLO_IP = "192.168.10.1"
        self.PORT = 8889
        self.BUFFER_SIZE = 1024
        self.VIDEO_PORT = 11111
        self.wifi_adapter_ip = wifi_adapter_ip
        
        # Create and bind the UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((wifi_adapter_ip, 0))
        
        # Get the port that was automatically selected
        self.host, self.port = self.sock.getsockname()
        logging.info(f"Socket bound to {self.host}:{self.port}")

        self.frame = None
        self.frame_lock = Lock()
        self.video_thread = None
        self.running = False

    # Function to send command to Tello drone
    def send_command(self, command, retries=5):
        for attempt in range(retries):
            try:
                self.sock.sendto(command.encode('utf-8'), (self.TELLO_IP, self.PORT))
                logging.info(f"Sent command to drone via {self.wifi_adapter_ip}: {command}")
                print(f"Sent command to drone via {self.wifi_adapter_ip}: {command}") #message to send command
                response, _ = self.sock.recvfrom(self)
                response_decoded = response.decode('utf8') #gets the respond
                logging.debug(f"Response: {response_decoded}")
                return response_decoded
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
        video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        video_socket.bind('0.0.0.0', 11111)
        self.running = True
        while self.running:
            try:
                packet, _ = self.sock.recvfrom(2048)

                if len(packet) == 0:
                    frame = np.array(bytearray(packet), dtype=np.uint8)
                    img = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                    if img is not None:
                    
                        cv2.imshow("Tello Stream", img)
                        logging.debug(f"Displaying video frame o")

                
            except Exception as e:
                logging.error(f"Error in video streaming: {e}")
                break

    def get_frame_read(self):
        return self.frame
    
    def emergency(self):
        self.send_command("emergency")

    #automatically turns stream on
    def start_video_thread(self):
        self.running = True
        self.video_thread = Thread(target=self.receive_video, args=())
        self.video_thread.start()

    def end(self):
        self.running = False
        if self.video_thread is not None:
            self.video_thread.join()
        self.sock.close()

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

    def rotateCCW(self, angle):
        self.send_command(f'ccw {abs(angle)}')

    def rotateCW(self, angle):
        self.send_command(f'cw {abs(angle)}')


    def getBattery(self):
        battery = self.get_command("battery?")
        return battery


    def getHeight(self):
        height = self.get_command("height?")
        return height
    
    def get_speed(self):
        speed_data = self.get_command("speed?")
        speed_values = speed_data.split()
        return speed_values[0]

    def get_AngularSpeed(self, startingyaw):
        imudata = self.get_command("attitude?")
        yawvalues = imudata.split()
        currentyaw = int(yawvalues[0])

        # Calculate the change in yaw
        yaw_difference = currentyaw - startingyaw

        # Handle potential yaw wraparound (like from 360° back to 0°)
        if yaw_difference > 180:
            yaw_difference -= 360
        elif yaw_difference < -180:
            yaw_difference += 360
        angular_speed1 = yaw_difference / 1  
        
        return angular_speed1, currentyaw
