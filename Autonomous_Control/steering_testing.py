import math
import json
import socket
import sys
import threading
import time
import matplotlib.pyplot as plt
from dataclasses import dataclass
import keyboard

# status variables
connected = False

# constants
HOST = '141.83.207.191' #"10.42.0.1" IP from jetson #'141.83.207.191' IP from Laptop
PORT = 2222

# negative left, positive right
steering_angle = 0.0

def disconnect(s: socket.socket):
    global connected

    print("Disconnecting...")
    # send escape command to stop
    s.send(bytes("escape", "utf-8"))
    # wait for answer
    s.recv(1024)

    s.close()
    connected = False

def connect() -> socket.socket:
    global connected

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:       
        print("Connecting...") 
        s.settimeout(20) # try to connect for 20 seconds 
        s.connect((HOST, PORT))
    except:
        print("Timeout, could not connect. Make sure Jetson awaits for connection.")
        s.close()
        connected = False
        return s
    
    # listen for handshake
    data = s.recv(1024).decode("utf-8")
    s.send(bytes("atsv_controller", "utf-8"))

    connected = data == "atsv_brain_welcome"
    return s

def extractData(data: str) -> dict:
    splitted = data.split(sep="|")

    if len(splitted) != 12:
        print(f"Not enough data send by ATRP! Number of data points sent: {len(splitted)}.")
        return None

    orderedData = {
        "command": splitted[0],
        "maxSpeed": float(splitted[1]),
        "steeringSteps": splitted[2],
        "sensorAngle": splitted[3],
        "sensorSpeed": float(splitted[4]),
        "cameraPos": splitted[5],
        "cameraOri": splitted[6],
        "cameraConfidence": splitted[7],
        "imuAcc": splitted[8],
        "imuGyr": splitted[9],
        "imuMag": splitted[10],
        "gps": json.loads(splitted[11])
    }

    # for now i am ignoring camera and imu data
    return orderedData

def commandLoop():
    global steering_angle
    command = str("_")

    if keyboard.is_pressed('a'):
        command = command + "_leftauto"
        steering_angle = -16.0
    elif keyboard.is_pressed('d'):
        command = command + "_rightauto"
        steering_angle = 16.0


    return command, steering_angle

def main(argv: list[str]):
    global steering_angle
    jetson = connect()

    while connected:
        command, steering_angle = commandLoop() 

        if command == "":
            disconnect(jetson)    
        
        commands = f"{command}|{steering_angle:.8f}"
        jetson.send(bytes(commands, "utf8"))

        # wait for data and decode
        data = jetson.recv(2048).decode("utf8")
        orderedData = extractData(data) 

        print("Command: %s | Steering: %f \r" % (command, steering_angle))
        #sys.stdout.write("Command: %s | Max Velocity: %d | Current Velocity: %d \r" % (orderedData["command"], orderedData["maxSpeed"], orderedData["sensorSpeed"]))
        #sys.stdout.flush()

        # socket communication and updates every 0.1 seconds
        time.sleep(0.1)
        
    sys.stdout.write("\n")
    print("Exiting...")

if __name__ == "__main__":
    main(sys.argv)