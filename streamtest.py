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
#from customtello import VideoProxyServer
from customtello import myTello

import path_planner
import tello_tracking
import collision
import logging
import platform
import subprocess
import avoid
import re



lock = threading.Lock()
# Define the IP addresses of the two Wi-Fi adapters
WIFI_ADAPTER_1_IP = "192.168.10.2"  # IP address of Wi-Fi Adapter 1 (connected to Drone 1)
WIFI_ADAPTER_2_IP = "192.168.10.3"  # IP address of Wi-Fi Adapter 2 (connected to Drone 2)

drone_ips = [WIFI_ADAPTER_1_IP, WIFI_ADAPTER_2_IP]
base_port = 11111
TELLO_IP = "192.168.10.1"

# Adapter names
WIFI_1 = "Wi-Fi"
WIFI_2 = "Wi-Fi 2"

def run_command(command):
    """Runs a system command and returns output."""
    try:
        result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        print(f"Error running command: {e}")
        return None

def get_connected_ssid(adapter):
    """Returns the SSID of the currently connected network for the given adapter."""
    system = platform.system()
    
    if system == "Windows":
        result = run_command('netsh wlan show interfaces')
        match = re.search(r"(?:IP Address|IPv4 Address)[\s.]+:\s+([\d.]+)", result)
        return match.group(1) if match else None
    elif system in ["Linux", "Darwin"]:
        result = run_command(f'nmcli -t -f active,ssid dev wifi | grep "^yes"')
        return result.strip().split(":")[-1] if result else None
    return None


def connect_wifi(adapter, ssid):
    """Connects Wi-Fi adapter to a specific Tello network only if not already connected."""
    current_ssid = get_connected_ssid(adapter)
    
    if current_ssid == ssid:
        print(f"{adapter} is already connected to {ssid}. Skipping connection.") 
    
    print(f"Connecting {adapter} to {ssid}...")
    system = platform.system()
    
    if system == "Windows":
        run_command(f'netsh wlan connect name="{ssid}" interface="{adapter}"')
    else:  # Linux/macOS
        run_command(f'nmcli device wifi connect "{ssid}" ifname {adapter}')
    
    time.sleep(2)



def get_current_ip(adapter):
    """Returns the current IP address of the Wi-Fi adapter."""
    system = platform.system()
    
    if system == "Windows":
        result = run_command(f'netsh interface ip show address name="{adapter}"')
        match = re.search(r"IP Address:\s+([\d.]+)", result)
        return match.group(1) if match else None
    elif system in ["Linux", "Darwin"]:
        result = run_command(f'ip addr show {adapter}')
        match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)/', result)
        return match.group(1) if match else None
    return None

def set_static_ip(adapter, ip):
    """Assigns a static IP to the Wi-Fi adapter only if it is not already set."""
    current_ip = get_current_ip(adapter)
    
    if current_ip == ip:
        print(f"Static IP for {adapter} is already {ip}. Skipping.")
        return
    
    print(f"Setting static IP {ip} for {adapter}...")
    system = platform.system()
    
    if system == "Windows":
        run_command(f'netsh interface ip set address name="{adapter}" static {ip} 255.255.255.0')
    else:  # Linux/macOS
        run_command(f'sudo ifconfig {adapter} {ip} netmask 255.255.255.0 up')


def check_route(ip):
    """Checks if a route already exists for the given IP."""
    system = platform.system()
    
    if system == "Windows":
        result = run_command("route print")
    elif system in ["Linux", "Darwin"]:
        result = run_command("ip route show")
    else:
        return False
    
    return ip in result if result else False

def add_route():
    """Adds routing rules only if they do not already exist for each adapter."""
    system = platform.system()

    route_1_exists = check_route(WIFI_ADAPTER_1_IP)
    route_2_exists = check_route(WIFI_ADAPTER_2_IP)

    if route_1_exists and route_2_exists:
        print(f"Routes for {TELLO_IP} already exist. Skipping.")
        return

    print(f"Adding missing route(s) for {TELLO_IP}...")

    if system == "Windows":
        if not route_1_exists:
            run_command(f'route -p add {TELLO_IP} mask 255.255.255.255 {WIFI_ADAPTER_1_IP} metric 1')
        if not route_2_exists:
            run_command(f'route -p add {TELLO_IP} mask 255.255.255.255 {WIFI_ADAPTER_2_IP} metric 1')
    elif system in ["Linux", "Darwin"]:
        if not route_1_exists:
            run_command(f'sudo ip route add {TELLO_IP} via {WIFI_ADAPTER_1_IP} dev wlan0')
        if not route_2_exists:
            run_command(f'sudo ip route add {TELLO_IP} via {WIFI_ADAPTER_2_IP} dev wlan1')

# Connect to Tello networks
connect_wifi(WIFI_1, "TELLO-D06F9F")

# Assign static IPs
set_static_ip(WIFI_1, WIFI_ADAPTER_1_IP)

# Add route if necessary
add_route()

#proxy_server = VideoProxyServer(drone_ips, server_ip, base_port)
#proxy_server.start_proxy()

drone1 = myTello(WIFI_ADAPTER_1_IP, base_port)

drone1.connect()
time.sleep(1)
print(drone1.getBattery())

drone1.streamon()
time.sleep(2)
drone1.start_video_thread(1)