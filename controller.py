import keyboard
import socket
import sys
import threading
import json

from inputs import get_gamepad
from inputs import UnpluggedError

from dataclasses import dataclass

# gamepad variables
joystickState = None

# status variables
useGamePad = False
connected = False

# constants
HOST = "10.42.0.1" # IP from jetson
PORT = 2222

@dataclass
class JoystickState:
    left_stick: int
    right_stick: int
    left_back_button: int
    right_back_button: int
    button_b: int
    button_back: int

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

    if len(splitted) != 11:
        print(f"Not enough data send by ATRP! Number of data points sent: {len(splitted)}.")
        return None

    orderedData = {
        "command": splitted[0],
        "maxSpeed": float(splitted[1]),
        "steeringSteps": json.loads(splitted[2]),
        "sensorAngle": int(splitted[3]),
        "sensorSpeed": float(splitted[4]),
        "cameraPos": json.loads(splitted[5]),
        "cameraOri": json.loads(splitted[6]),
        "cameraConfidence": float(splitted[7]),
        "imuAcc": json.loads(splitted[8]),
        "imuGyr": json.loads(splitted[9]),
        "imuMag": json.loads(splitted[10])
    }

    # for now i am ignoring camera and imu data
    return orderedData

def commandLoop(useGamePad: bool) -> str:
    command = str("")

    if useGamePad:
        global joystickState
        #print("Events: ", joystickEvents)

        #print(event.code, event.state)
        if joystickState.button_back == 1:
            return command


        if joystickState.right_stick < 70:
            command = command + "_left"
        elif joystickState.right_stick > 200:
            command = command + "_right"


        if joystickState.left_back_button == 1:
            command = command + "_deccelerate"
        elif joystickState.right_back_button == 1:
            command = command + "_accelerate"


        if joystickState.left_stick == -1:
            command = command + "_forward"
        elif joystickState.left_stick == 1:
            command = command + "_backward"
        elif joystickState.button_b == 1:
            command = command + "_stop"
        else:
            command = command + "_nothing"
    else:        
        if keyboard.is_pressed("Escape"):
            return command
        
        if keyboard.is_pressed('A'):
            command = command + "_left"
        elif keyboard.is_pressed('D'):
            command = command + "_right"

        if keyboard.is_pressed('Ctrl'):
            command = command + "_deccelerate"
        elif keyboard.is_pressed('Shift'):
            command = command + "_accelerate"

        if keyboard.is_pressed('W'):
            command = command + "_forward"
        elif keyboard.is_pressed('S'):
            command = command + "_backward"
        elif keyboard.is_pressed('Space'):
            command = command + "_stop"
        else:
            command = command + "_nothing"

    return command

def monitorGamepad():
    global joystickState
    while True:
        events = []
        try:
            events = get_gamepad()
        except UnpluggedError:
            print("No gamepad Found! Use keyboard!")
            break

        for event in events:
            if event.code == "BTN_TL2":
                joystickState.button_back = event.state

            if event.code == "ABS_Z":
                joystickState.right_stick = event.state


            if event.code == "BTN_TL":
                joystickState.left_back_button = event.state
            elif event.code == "BTN_TR":
                joystickState.right_back_button = event.state


            if event.code == "ABS_HAT0Y":
                joystickState.left_stick = event.state
            elif event.code == "BTN_C":
                joystickState.button_b = event.state


    global useGamePad
    useGamePad = False
    

def main(argv: list[str]):    
    global useGamePad
    global joystickState

    for _, arg in enumerate(argv):
        if "gamepad" in arg:
            joystickState = JoystickState(0, 125, 0, 0, 0, 0)

            # start gamepad thread
            monitorThread = threading.Thread(target=monitorGamepad, args=())
            monitorThread.daemon = True
            monitorThread.start()

            useGamePad = True


    jetson = connect()

    # start loop
    while connected:
        command = commandLoop(useGamePad) 

        if command == "":
            disconnect(jetson)    
        
        # send command
        jetson.send(bytes(command, "utf8"))

        # wait for data and decode
        data = jetson.recv(2048).decode("utf8")
        orderedData = extractData(data)

        if not orderedData is None:
            sys.stdout.write("Max Velocity: %d | Current Velocity: %d \r" % (orderedData["maxSpeed"], orderedData["sensorSpeed"]))
            sys.stdout.flush()

    sys.stdout.write("\n")
    print("Exiting...")

if __name__ == "__main__":
    main(sys.argv)