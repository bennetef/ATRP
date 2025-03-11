"""ROS2 ATRP driver."""

import rclpy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from rclpy.node import Node
import socket
import time
import threading
import json
import sys
import signal

HOST = '192.168.178.52'
PORT = 2222

status = "stop_notCalibrated"
ackermann_message = AckermannDrive()
speed = 10.0    # kmh!
maxSpeed = 10.00 # kmh
steering_angle = 0.0
gps = [0.0, 0.0]
controller = None
gps_speed = 0.0 # m/s
clientSocket = None
shutdown_event = threading.Event()

class ATRPController(Node):
    def __init__(self):
        super().__init__('ATRP_webots_controller')

        # ROS interface
        self.__ackermann_publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 1)
        self.create_subscription(NavSatFix, 'gps', self.gps_callback, 1)
        self.create_subscription(Float32, 'gps/speed', self.gps_speed_callback, 1)
        self.get_logger().info('ATRPController node has been initialized.')

    def gps_callback(self, message):
        global gps
        gps = [message.latitude, message.longitude]
        # self.get_logger().info(f'GPS: {gps}')

    def gps_speed_callback(self, message):
        global gps_speed
        gps_speed = message.data

    def change_direction(self, speed):
        global ackermann_message
        ackermann_message.speed = speed

        self.__ackermann_publisher.publish(ackermann_message)

    def change_maxSpeed(self, variant):
        global maxSpeed
        if variant == 1:
            newMaxSpeed = maxSpeed + 0.0139   # +/- 0.05 km/h
        elif variant == -1:
            newMaxSpeed = maxSpeed - 0.0139

        maxSpeed = newMaxSpeed

        if newMaxSpeed > 5.5:
            maxSpeed = 5.5
        if newMaxSpeed < 1.39:
            maxSpeed = 1.39

    def accelerate(self):
        global speed
        global maxSpeed

        if speed < maxSpeed:
            speed = speed + 0.05556   # +/- 0.2 km/h
        if speed > maxSpeed:
            speed = speed - 0.05556

    def change_steering(self, direction):
        global ackermann_message
        global steering_angle
        max_steering_angle = 0.279253
        
        if direction == 0: # left
            if steering_angle - 0.0004363328125 > -max_steering_angle:
                steering_angle = steering_angle - 0.0004363328125   # +/- 0.025 degrees
            else:
                steering_angle = -max_steering_angle
        elif direction == 1: # right
            if steering_angle + 0.0004363328125 < max_steering_angle:
                steering_angle = steering_angle + 0.0004363328125
            else:
                steering_angle = max_steering_angle

        ackermann_message.steering_angle = steering_angle
        self.__ackermann_publisher.publish(ackermann_message)

    def steering_to_angle(self, steering):
        global ackermann_message

        steering = steering * (3.141592653589793 / 180.0)  # Convert degrees to radians

        ackermann_message.steering_angle = steering
        self.__ackermann_publisher.publish(ackermann_message)

    def stop(self, stopTime):
        global ackermann_message
        ackermann_message.speed = 0.0

        self.__ackermann_publisher.publish(ackermann_message)

        time.sleep(stopTime)
    
def socket_communication():

    # Create a new socket for IPv4 addresses
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #reuse the socket
    # Hosting with given IP and Port
    s.bind((HOST, PORT))
    # Setting up
    s.listen(1) # only allow 1 default connection
    s.settimeout(120) # listen for 2 min then close

    global status
    global ackermann_message
    global speed
    global maxSpeed
    global controller
    global gps
    global gps_speed
    global clientSocket

    try:
        while not shutdown_event.is_set():  # for client connection
            try:
                print("Waiting for Connection of Controller on:", str(s))
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
                print("Connected to valid controller!")
            else:
                # Close socket and wait for another connection
                clientSocket.close()
                continue

            while not shutdown_event.is_set():  # for continuous communication
                try:
                    """Response to client
                        
                    After each cycle of the loop, the vehicle sends data to the client.
                    The status and other information is send via the TCP connection.
                    """
                    clientSocket.send(bytes(status + "|" + str(maxSpeed) + "|" +
                                            "steeringSteps" + "|" +
                                            "sensorAngle" + "|" +  
                                            str(gps_speed) + "|" +
                                            "camPos" + "|" +
                                            "camOri" + "|" +
                                            "confidence" + "|" +
                                            "imu.getAcceleration" + "|" +
                                            "imu.getAngularRate" + "|" +
                                            "imu.getMagneticField" + "|" +
                                            json.dumps(gps), "utf-8"))
                    
                    """Read Command from controller

                    controller can send following commands:
                    - forward: The forward key is pressed
                    - backward: The backward key is pressed
                    - accelerate: The acceleration key is pressed
                    - deccelerate: the decceleration key is pressed
                    - stop: The Break key is pressed (stops motor)
                    - left: steer Left key is pressed
                    - right: steer right key is pressed
                    - escape: terminates the program and shuts robot off
                    - nothing: no key is pressed
                    An example command string: "_forward_left"
                    """
                    commands = clientSocket.recv(1024).decode("utf-8")
                    command = commands.split("|")[0]
                    steering = float(commands.split("|")[1])
                    print(command)

                    # ESCAPING
                    # Stop vehicle and exit program
                    if "escape" in command:
                        controller.stop(2)  # find a way to close webots 
                        break

                    # ACCELERATION / DECCELERATION 
                    # Change max speed value by fixed amount
                    if "deccelerate" in command:
                        controller.change_maxSpeed(-1)
                    elif "accelerate" in command:
                        controller.change_maxSpeed(1)

                    # LEFT / RIGHT
                    # Change steering angle by fixed amount
                    if "left" in command: # If commanded to go left
                        controller.change_steering(0)

                        # Update status accordingly           
                        status = status.split("_")[0] + "_left"
                    elif "right" in command: # If commanded to go right
                        controller.change_steering(1)

                        # Update status accordingly
                        status = status.split("_")[0] + "_right"

                    if "lauto" in command:
                        controller.steering_to_angle(steering)
                        status = status.split("_")[0] + "_lauto"

                    elif "rauto" in command:
                        controller.steering_to_angle(steering)
                        status = status.split("_")[0] + "_rauto"

                    # FORWARD / BACKWARD / STOP
                    if "fauto" in command:
                        controller.change_direction(speed)
                        status = status.split("_")[0] + "_fauto"
                    elif "bauto" in command:
                        controller.change_direction(-speed)
                        status = status.split("_")[0] + "_bauto"

                    if "forward" in command:
                        # Check if vehicle went backward before so wait for 0.3 seconds to give the hardware breathing room
                        if "backward" in status:
                            controller.stop(0.3)
                            speed = 1.39
                        # Adjust speed accordingly
                        controller.accelerate()
                        controller.change_direction(speed)
                        status = "forward_" + status.split("_")[1]

                    elif "backward" in command:
                        # Check if vehicle went forward before so wait for 0.3 seconds to give the hardware breathing room
                        if "forward" in status:
                            controller.stop(0.3)
                            speed = 1.39
                        # Adjust speed accordingly
                        controller.accelerate()
                        controller.change_direction(-speed)
                        status = "backward_" + status.split("_")[1]

                    elif "stop" in command:
                        # Stop vehicle for atleast 2 seconds
                        speed = 1.39
                        controller.stop(2)
                        status = "stop_" + status.split("_")[1]

                    elif "nothing" in command:
                        # If nothing is commanded robot stops and speed value resets
                        controller.stop(0)
                        speed = 1.39
                        status = "stop_" + status.split("_")[1]

                except (socket.error, ConnectionResetError):
                    break

    finally:
        if clientSocket:
            clientSocket.close()
        s.close()
        print("Socket communication terminated")


def main(args=None):
    global controller

    try:
        rclpy.init(args=args)
        controller = ATRPController()
        
        socketThread = threading.Thread(target=socket_communication)
        socketThread.daemon = True
        socketThread.start()
        
        try:
            rclpy.spin(controller)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            print("Shutting down")
        finally:
            shutdown_event.set()
            socketThread.join(timeout=5)
            if socketThread.is_alive():
                print("Warning: Socket thread did not terminate cleanly")
            try:
                controller.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during rclpy shutdown: {e}")
        sys.exit(0)


if __name__ == '__main__':
    main()