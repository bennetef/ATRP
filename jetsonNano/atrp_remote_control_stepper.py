"""ATSV Remote Control 

This script allows the user to remote control the AT-SV. Furthermore it can be
seen as a demo to show users how to interact with the hardware of the vehicle. 
This can be used to implement autonomous and intellegent features. 

For this script to work some modules are required. Be sure to install them
after resetting the OS on the Jetson Nano.

--Jetson Linux Account--
User: AT-SV
Password: atsv
"""
# Mathy Libraries
import numpy as np

# Used for communication
import socket
import smbus
import json

# For GPS module
from gps3 import agps3
import coordTransform_py.coordTransform_utils as transform

# ZED Camera Library
import pyzed.sl as sl

# Essentials
import time
import RPi.GPIO as GPIO

# IMU scripts
from imu_phidget_spatial import IMU

"""Defining GPIO pins
    
Note that the Jetson uses different GPIO mappings. In this script we are using TEGRA_SOC.
"""
relay_plus_forward_pin = 'DAP4_SCLK'  # BOARD pin 12, BCM pin 18 (GELB)
relay_minus_forward_pin = 'SPI2_CS1' # BOARD pin 16, BCM pin 23 (LILA)
relay_plus_backward_pin = 'SPI2_CS0' # BOARD pin 18, BCM pin 24 (BLAU)
relay_minus_backward_pin = 'SPI2_MISO' # BOARD pin 22, BCM pin 25 (ORANGE)
da_converter_throttle_pin = 'LCD_BL_PW'
relay_nano_steering_pin = 'UART2_RTS' # BOAD pin 11, BCM pin 17 (GRAY)

# Pins for direction
steering_left_pin = 'SPI1_MOSI' # BOARD pin 19, BCM pin 10
steering_right_pin = 'SPI1_MISO' # BOARD pin 21, BCM pin 9

# Setting up I2C bus 0 (Pins 27, 28) for sensor arduino comms
arduino_sensor = smbus.SMBus(0)
i2cAddress_sensor = 0x40

# Setting up I2C bus 1 (Pins 3, 5) for steüüer arduino comms 
arduino_stepper = smbus.SMBus(1)
i2cAddress_stepper = 0x60

# Steering sensor and translation vector old value
sensorAngleOld = 0
oldTranslationVector = np.zeros((3))

HOST = '10.42.0.1' # IP from Jetson Nano
PORT = 2222 # Some Number with 4 digits

# Initialize parameters for zed camera
initParams = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                            coordinate_units=sl.UNIT.METER,
                            coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
runTimeParams = sl.RuntimeParameters()

# Create camera object
zed = sl.Camera()
cameraSuccessful = False

if zed.open(initParams) != sl.ERROR_CODE.SUCCESS:
   print("ZED CAMERA DID NOT INITIALZE")
else:
    cameraSuccessful = True

"""Status Variable

This variable shows the current status of the vehicle. To allow multiple stati at once we concatenate 
strings seperated by '_'. This allows us to expand the status further and gives the controller insight in what is
going on with the vehicle. 

stati: stop, forward, backward, notCalibrate, left, right

Note: "left" and "right" stati represent the side on which the steering servo has contact
"""
status = "stop_notCalibrated"

def stop_main_motor(stopTime: int):
    """Stops the main Motor

    Args:
        stopTime (int): time to wait before continuing to accept inputs
        
    This method sets the relais, so that the voltage over the main motor is 0.
    """
    GPIO.output(relay_plus_forward_pin, GPIO.HIGH)
    GPIO.output(relay_minus_forward_pin, GPIO.HIGH)
    GPIO.output(relay_plus_backward_pin, GPIO.HIGH)
    GPIO.output(relay_minus_backward_pin, GPIO.HIGH)
    
    time.sleep(stopTime)

def accelerate_to_maxspeed(acceleration: float):
    """Accelerates to given max speed
    
    Args:
        acceleration (float): change in speed for each tick
    
    The speed value corresponds to the duty cycle of the pwm signal, that drives the
    da-converter. This is changed in this method by a given amount.
    """
    global pwmSignal
    global speedValue
    global maxSpeedValue

    # Change the speed value depending on the set max speed
    if speedValue < maxSpeedValue:
        speedValue = speedValue + acceleration
    if speedValue > maxSpeedValue:
        speedValue = speedValue - acceleration

    # Write new duty cycle percentage to pwm signal 
    pwmSignal.ChangeDutyCycle(speedValue)

def hold_forward():
    """Gets called each tick when commanding forward
    
    Each tick the controller commands to go forward the vehicle accelerates to the set max speed. 
    Also the relais are set to apply a negative voltage over the main motor to achieve a forward rotation.
    """
    accelerate_to_maxspeed(0.05)

    # Set GPIO outputs for the relais
    GPIO.output(relay_plus_forward_pin, GPIO.LOW)
    GPIO.output(relay_minus_forward_pin, GPIO.LOW)
    
    # Update status for the robot
    global status
    status = "forward_" + status.split("_")[1]

def hold_backward():
    """Gets called each tick when commanding backward
    
    Each tick the controller commands to go backward the vehicle accelerates to the set max speed. 
    Also the relais are set to apply a positive voltage over the main motor to achieve a backward rotation.
    """
    accelerate_to_maxspeed(0.05)

    # Set GPIO outputs for the relais
    GPIO.output(relay_plus_backward_pin, GPIO.LOW)
    GPIO.output(relay_minus_backward_pin, GPIO.LOW)
    
    # Update status for the robot
    global status
    status = "backward_" + status.split("_")[1]

def changeSteering(direction: int):
    """Change stepper in one particular directions
    
    Args:
        direction: Integer that indicates the direction  of movement. 0 is left while 1 is to the right.
    """
    if direction == 0:
        GPIO.output(steering_left_pin, GPIO.HIGH)
        GPIO.output(steering_right_pin, GPIO.LOW)
    elif direction == 1:
        GPIO.output(steering_right_pin, GPIO.HIGH)
        GPIO.output(steering_left_pin, GPIO.LOW)
    else:
        GPIO.output(steering_right_pin, GPIO.LOW)
        GPIO.output(steering_left_pin, GPIO.LOW)

def changeMaxSpeed(newMaxSpeedValue):
    """Constrains max speed
    
    Args:
        newMaxSpeedValue: the new max speed value
        
    This method is called to change the max speed so it is constrained to its max and min values
    """
    global maxSpeedValue

    # Set the new value
    maxSpeedValue = newMaxSpeedValue

    # Constrain the new value
    if maxSpeedValue > 36:
        maxSpeedValue = 36
    if maxSpeedValue < 21:
        maxSpeedValue = 21

def filterSensorAngle(sensorAngle): 
    """This method filters out peaks

    Args:
        sensorAngle: the measured angle

    Returns:
        the filtered value
    """
    
    dif = abs(sensorAngle - sensorAngleOld)
    
    if dif > 15:
        sensorAngle = sensorAngleOld
        
    return sensorAngle
      
def getCameraPosition(cameraPose):
    """This method returns the camera position of the ZED camera
    
    Return:
        Numpy array containing the position as vector and the
        difference to last pos as scalar
    """
    if cameraPose is None:
        return (0, 0, 0)

    global oldTranslationVector
    global cameraSuccessful

    if zed.grab(runTimeParams) == sl.ERROR_CODE.SUCCESS:
        positionalState = zed.get_position(cameraPose, reference_frame=sl.REFERENCE_FRAME.WORLD)

        #if positionalState == sl.POSITIONAL_TRACKING_STATE.OK:
        translation = sl.Translation()
        translationVector = cameraPose.get_translation(translation).get()

        difVector = translationVector - oldTranslationVector
        oldTranslationVector = translationVector
            
        orientation = sl.Orientation()
        orientationVector = cameraPose.get_orientation(orientation).get()
            
        confidence = cameraPose.pose_confidence

        return (np.append(translationVector, np.linalg.norm(difVector)).tolist(), orientationVector.tolist(), confidence)
        #else:
        #    print("Positional Tracking State not ok!")
        #    return np.append(oldTranslationVector, 0.0, 0).tolist()
    else:
        return (np.append(oldTranslationVector, 0.0, 0).tolist(), 0.0, 0.0)

def initCamera(magneticField, acceleration):  
    # Calculate Psi
    psi = np.arctan2(magneticField[1], magneticField[2])
    psi = psi if psi > 0 else psi + 2*np.pi

    # Enable Positional Tracking
    trackingParams = sl.PositionalTrackingParameters(sl.Transform(
                        sl.Rotation(0.0, psi - np.pi/2, 0.0), 
                        sl.Translation(0.0, 0.0, 0.0)))
    zed.enable_positional_tracking(trackingParams)
    # Get camera pose
    return sl.Pose()

def main():
    """Main function of this program
    
    This represents the starting point of the script. 
    """
    # Pin Setup:
    # Print board pin-numbering scheme
    GPIO.setmode(GPIO.TEGRA_SOC)
    print('GPIO Mode:', GPIO.getmode())
    GPIO.setwarnings(False)

    # Initialize used GPIO pins 
    # Pins for the relais
    GPIO.setup(relay_plus_forward_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(relay_minus_forward_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(relay_plus_backward_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(relay_minus_backward_pin, GPIO.OUT, initial=GPIO.HIGH)
    # PWM Pin for DA-Converter to control the throttle
    GPIO.setup(da_converter_throttle_pin, GPIO.OUT, initial=GPIO.LOW)
    
    # Steering Pins
    GPIO.setup(steering_left_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(steering_right_pin, GPIO.OUT, initial=GPIO.LOW)

    # Create imu object
    imu = IMU(5000)

    # Wait for Hardware to initialize 
    time.sleep(3)
    
    #------------------------ AUDIBLE CLICK OF THE RELAIS ----------------------------------

    # Create a new socket for IPv4 addresses
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Hosting with given IP and Port
    s.bind((HOST, PORT))
    # Setting up
    s.listen(1) # only allow 1 default connection
    s.settimeout(120) # listen for 2 min then close

    # Declare global variables
    global status
    global pwmSignal
    global speedValue
    global maxSpeedValue
    
    # Initializing variables
    speedValue = 21 # Lowest speed value
    maxSpeedValue = 21 # Lowest speed value

    # Start PWM signal
    pwmSignal = GPIO.PWM(da_converter_throttle_pin, 100)
    pwmSignal.start(speedValue) # The speed value corresponds to the duty cycle of the PWM signal

    # Get inital camera pose
    if cameraSuccessful:
        cameraPose = initCamera(imu.getMagneticField(), imu.getAcceleration())
    else:
        cameraPose = None

    # Accept first connect338646 ions in outer loop
    while True:
        try:
            print("Wating for Connection of Controller on:", str(s))
            clientSocket, address = s.accept()
        except socket.timeout:
            """Timeout leads to exiting program
            
            This behaviour might not be desired. Change if necessary.
            """
            print("timeout: Exiting Program...")
            return

        print('Connection has been established:', address)
        # Send handshake information to client 
        clientSocket.send(bytes("atsv_brain_welcome", "utf-8"))
        # Check client
        message = clientSocket.recv(1024).decode("utf-8")
        if message == "atsv_controller":
            # Connected to valid controller
            print("connected to valid controller!")
        else:
            # Close socket and wait for another connection
            clientSocket.close()
            continue

        """MAIN LOOP
        
        This loop is responsible for controlling the vehicle by reading the 
        commands send from the connected client.
        Sensor data received over the i2c connection with the arduino board
        is read and used for optimising control.
        """
        while True:
            # Arduino communication 
            steeringSteps = arduino_stepper.read_i2c_block_data(i2cAddress_stepper, 0, 2)

            # Read 2 bytes block from given address starting at register 0 
            [sensorSpeed, sensorAngle] = arduino_sensor.read_i2c_block_data(i2cAddress_sensor, 0, 2)
            #sensorSpeed = 0
            #sensorAngle = 0
            # Convert detected speed value to m/s it's in m/0.25s
            sensorSpeed = 4 * sensorSpeed / 10
            # Convert detected steering angle to percent (negative: right, positve: left)
            if sensorAngle > 130:
                sensorAngle = sensorAngle - 250
               
            # Filter out peaks from measured value
            sensorAngle = filterSensorAngle(sensorAngle) 
            # Convert percentage to degrees            
            sensorAngleDegrees = 120 + (sensorAngle / 100 * 13)

            """Read Command from Client

            Client can send following commands:
            - forward: The forward key is pressed
            - backward: The backward key is pressed
            - accelerate: The acceleration key is pressed
            - deccelerate: the decceleration key is pressed
            - stop: The Break key is pressed (stops motor)
            - left: steer Left key is pressed
            - right: steer right key is pressed
            - escape: terminates the program and shuts robot off
            - nothing: no key is pressed
            An example command string: "forward-left"
            """
            command = clientSocket.recv(1024).decode("utf-8")
            print(command)

            # ESCAPING
            # Stop vehicle and exit program
            if "escape" in command:
                stop_main_motor(2)
                break

            # ACCELERATION / DECCELERATION 
            # Change max speed value by fixed amount
            if "deccelerate" in command:
                changeMaxSpeed(maxSpeedValue - 0.05)
            elif "accelerate" in command:
                changeMaxSpeed(maxSpeedValue + 0.05)

            # LEFT / RIGHT
            # Change steering angle by fixed amount
            if "left" in command: # If commanded to go left                  
                changeSteering(0)   

                # Update status accordingly           
                status = status.split("_")[0] + "_left"
            elif "right" in command: # If commanded to go right
                changeSteering(1)

                # Update status accordingly
                status = status.split("_")[0] + "_right"
            else:
                # No steering command, set both to LOW
                changeSteering(-1)
                    

            # FORWARD / BACKWARD / STOP
            if "forward" in command:
                # Check if vehicle went backward before so wait for 0.3 seconds to give the hardware breathing room
                if "backward" in status:
                    stop_main_motor(0.3)
                    speedValue = 21
                # Adjust speed accordingly
                hold_forward()
            elif "backward" in command:
                # Check if vehicle went forward before so wait for 0.3 seconds to give the hardware breathing room
                if "forward" in status:
                    stop_main_motor(0.3)
                    speedValue = 21
                # Adjust speed accordingly
                hold_backward()
            elif "stop" in command:
                # Stop vehicle for atleast 2 seconds
                speedValue = 21
                stop_main_motor(2)
                status = "stop_" + status.split("_")[1]
            elif "nothing" in command:
                # If nothing is commanded robot stops and speed value resets
                stop_main_motor(0)
                # Reset speed depending on current measured speed
                speedValue = 19 + (sensorSpeed * 1.325)
                status = "stop_" + status.split("_")[1]


            """Response to client
            
            After each cycle of the loop, the vehicle sends data to the client.
            The status and other information is send via the TCP connection.
            """
            (camPos, camOri, confidence) = getCameraPosition(cameraPose)
            clientSocket.send(bytes(status + "|" + 
                                    str(maxSpeedValue) + "|" + 
                                    str(steeringSteps) + "|" +
                                    str(sensorAngle) + "|" +  
                                    str(sensorSpeed) + "|" +
                                    json.dumps(camPos) + "|" +
                                    json.dumps(camOri) + "|" +
                                    json.dumps(confidence) + "|" +
                                    json.dumps(imu.getAcceleration()) + "|" +
                                    json.dumps(imu.getAngularRate()) + "|" +
                                    json.dumps(imu.getMagneticField()), "utf-8"))

        # Before exiting main loop send last statement to client and close socket
        clientSocket.send(bytes("closing", "utf-8"))
        time.sleep(0.5)
        clientSocket.close()
        break

    # Clean up GPIO Pins and exit program
    GPIO.cleanup()
    quit()

if __name__ == "__main__":
    main()
