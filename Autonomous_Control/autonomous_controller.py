import math
import json
import socket
import sys
import threading
import time
import keyboard
import csv
import datetime
import matplotlib.pyplot as plt
from dataclasses import dataclass
import numpy as np
from kalman_filter import KalmanFilterGPS, KalmanFilterParams
from pid_controller import PIDController, PIDParams

# status variables
connected = False

# constants
HOST = '192.168.178.52' #"10.42.0.1" IP from jetson #'192.168.178.52' IP from Laptop
PORT = 2222

# Start GPS coordinates (latitude, longitude)
start_coordinates = [53.834159, 10.697536]
end_coordinates = [53.834306, 10.698006]

# S-Shape coordinates
s_start = [53.834180, 10.697603]
s_mid = [53.834253, 10.697881]
s_end = [53.834326, 10.698159]

# negative left, positive right
steering_angle = 0.0

path = []

# Straight line path
line_path = [   [53.834159, 10.697536],
                [53.834180, 10.697603],
                [53.834201, 10.697670],
                [53.834222, 10.697737],
                [53.834243, 10.697805],
                [53.834264, 10.697872],
                [53.834285, 10.697939],
                [53.834306, 10.698006]]

curve_path = [ ]

s_path = []

test_path = [   [53.834159, 10.697536],
                [53.834180, 10.697603],
                [53.834201, 10.697670],
                [53.834222, 10.697737],
                [53.834209, 10.697771], # step to the right
                [53.834230, 10.697838],
                [53.834251, 10.697905],
                [53.834272, 10.697972],
                [53.834293, 10.698039],
                [53.834314, 10.698106],
                [53.834335, 10.698173]]

def disconnect(s: socket.socket):
    global connected

    print("Disconnecting...")
    # send escape command to stop
    s.send(bytes("escape", "utf-8"))
    # wait for answer
    s.recv(1024)

    s.close()
    connected = False
    return

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
        "gps_speed": float(splitted[4]),
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

def gps_to_meters(lat, lon, start_lat, start_lon):
    """Convert GPS coordinates to local ENU coordinates."""
    earth_radius = 6378137  # Radius of Earth in meters
    dlat = math.radians(lat - start_lat)
    dlon = math.radians(lon - start_lon)
    mean_lat = math.radians((lat + start_lat) / 2.0)
    
    x = dlon * earth_radius * math.cos(mean_lat)  # East
    y = dlat * earth_radius  # North

    #print(f"X: {x}, Y: {y}")
    # print(f"GPS to meters: lat={lat}, lon={lon}, start_lat={start_lat}, start_lon={start_lon}, x={x}, y={y}")
    return x, y

def cross_track_error(lat_v, lon_v, lat_start, lon_start, lat_n, lon_n):
    """Calculate cross-track error (perpendicular distance from vehicle to path A->B)."""
    # Convert GPS coordinates to meters relative to Start
    x_start, y_start = 0, 0 # Start point
    x_v, y_v = gps_to_meters(lat_v, lon_v, lat_start, lon_start) # Vehicle
    x_n, y_n = gps_to_meters(lat_n, lon_n, lat_start, lon_start) # next path point
    
    # Shortest distance from vehicle to path Last Point->Next point
    error = abs((y_n - y_start) * x_v - (x_n - x_start) * y_v + x_n * y_start - y_n * x_start) / math.sqrt((y_n - y_start)**2 + (x_n - x_start)**2)
    
    # Determine the sign (negativ:left of path/positiv:right of path) using the cross-product
    sign = math.copysign(1, (x_n - x_start) * (y_v - y_start) - (y_n - y_start) * (x_v - x_start))
    
    # print(f"Cross-track error: lat_v={lat_v}, lon_v={lon_v}, lat_start={lat_start}, lon_start={lon_start}, lat_n={lat_n}, lon_n={lon_n}, error={error}, sign={sign}")
    return sign * error, x_v, y_v, x_n, y_n

def heading_error(x_v, y_v, x_n, y_n, x_l, y_l):
    """Calculate heading error (angle between vehicle heading and path)."""
    # Calculate the heading of last position to path
    path_heading = math.atan2(y_n - y_l, x_n - x_l)
    
    # Calculate the heading of the vehicle to path
    vehicle_heading = math.atan2(y_v - y_l, x_v - x_l)
    
    # Calculate the heading error
    error = vehicle_heading - path_heading
    
    # Normalize the error to the range -pi to pi
    if error > math.pi:
        error -= 2 * math.pi
    elif error < -math.pi:
        error += 2 * math.pi
    
    # print(f"Heading error: x_v={x_v}, y_v={y_v}, x_n={x_n}, y_n={y_n}, x_l={x_l}, y_l={y_l}, path_heading={path_heading}, vehicle_heading={vehicle_heading}, error={error}")
    return error

def error_to_steering(error):
    """Map error proportional to steering angle."""
    max_steering_angle = 16  # Maximum steering angle in degrees
    max_error = 3  # Maximum error
    steering_angle = max_steering_angle * error / max_error
    return steering_angle

def create_curve_path():
    lat_start, lon_start = [53.834180, 10.697603]
    lat_end, lon_end = end_coordinates

    # Number of waypoints
    num_waypoints = 12

    # Parameter t ranging from 0 to 1
    t_values = np.linspace(0, 1, num_waypoints)

    # Define curvature strength (adjustable)
    A = 0.00002  # Small vertical deviation

    # Compute latitudes and longitudes
    latitudes = lat_start + t_values * (lat_end - lat_start) + A * np.sin(np.pi * t_values)
    longitudes = lon_start + t_values * (lon_end - lon_start)

    # Combine into waypoints
    waypoints = list(zip(latitudes, longitudes))

    # Add the generated waypoints to the curve_path
    global curve_path
    curve_path = [start_coordinates]
    curve_path.extend(waypoints)

def create_s_shape_path(start, mid, end):
    """Create an S-shaped path"""
    num_waypoints = 20

    t_values = np.linspace(0, 1, num_waypoints // 2)
    
    # Define curvature strength (adjustable)
    A = 0.00002  # Small vertical deviation

    # Compute latitudes and longitudes
    latitudes1 = start[0] + t_values * (mid[0] - start[0]) + A * np.sin(np.pi * t_values)
    longitudes1 = start[1] + t_values * (mid[1] - start[1])
    
    # Compute latitudes and longitudes
    latitudes2 = mid[0] + t_values * (end[0] - mid[0]) - A * np.sin(np.pi * t_values)
    longitudes2 = mid[1] + t_values * (end[1] - mid[1])
    
    # Combine the two curves
    latitudes1 = latitudes1[:-1]    # delete duplicate at the junction
    longitudes1 = longitudes1[:-1]
    latitudes = np.concatenate((latitudes1, latitudes2))
    longitudes = np.concatenate((longitudes1, longitudes2))
    
    # Combine into waypoints
    waypoints = list(zip(latitudes, longitudes))
    
    global s_path
    s_path = [start_coordinates]
    s_path.extend(waypoints)

def commandLoop(pid: PIDController, path: list, current_gps: list, waypoint_index: int, last_position: list) -> (str, int):
    command = str("")

    # Get the current and next waypoints
    lat_v, lon_v = current_gps
    lat_start, lon_start = path[waypoint_index]
    lat_n, lon_n = path[waypoint_index + 1]
    #if waypoint_index + 2 >= len(path):
    #    lat_n, lon_n = path[waypoint_index + 1]
    #else:
    #    lat_n, lon_n = path[waypoint_index + 2]

    # Calculate the cross-track and heading error
    position_error, x_v, y_v, x_n, y_n = cross_track_error(lat_v, lon_v, lat_start, lon_start, lat_n, lon_n)
    x_l, y_l = gps_to_meters(last_position[0], last_position[1], lat_start, lon_start)
    heading_err = heading_error(x_v, y_v, x_n, y_n, x_l, y_l)

    error = 0.7*position_error + 0.3*heading_err

    steering_error = error_to_steering(error)

    # Update the PID controller
    dt = 1  # Time step
    error_correction = pid.update(steering_error, dt)

    # Determine the steering command based on the PID output
    if error_correction < 0:
        command = command + "_lauto"
    elif error_correction > 0:
        command = command + "_rauto"

    # Add forward command to keep moving
    command = command + "_fauto"

    # Check if the vehicle has reached the next waypoint
    distance_to_next_waypoint = math.sqrt((x_v - x_n)**2 + (y_v - y_n)**2)
    if distance_to_next_waypoint < 5.0:
        waypoint_index += 1

    return command, waypoint_index, error, error_correction, [lat_v, lon_v], steering_error

def main(argv: list[str]):
    global path

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")

    for _, arg in enumerate(argv):
        if "test" in arg:
            path = test_path
            path_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_test_path_coordinates_{timestamp}.csv'  #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\test_path_coordinates_{timestamp}.csv'
            vehicle_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_test_vehicle_coordinates_{timestamp}.csv'    #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\test_vehicle_coordinates_{timestamp}.csv'
            output_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_test_pid_output_{timestamp}.svg'     #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\test_output_{timestamp}.svg'
        elif "line" in arg:
            path = line_path
            path_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_line_path_coordinates_{timestamp}.csv'  #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\line_path_coordinates{timestamp}.csv'
            vehicle_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_line_vehicle_coordinates_{timestamp}.csv'   #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\line_vehicle_coordinates{timestamp}.csv'
            output_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_line_pid_output_{timestamp}.svg'     #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\line_output{timestamp}.svg'
        elif "curve" in arg:
            create_curve_path()
            path = curve_path
            path_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_curve_path_coordinates_{timestamp}.csv'   #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\curve_path_coordinates{timestamp}.csv'
            vehicle_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_curve_vehicle_coordinates_{timestamp}.csv'   #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\curve_vehicle_coordinates{timestamp}.csv'
            output_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_curve_pid_output_{timestamp}.svg'     #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\curve_output{timestamp}.svg'
        elif "s" in arg:
            create_s_shape_path(s_start, s_mid, s_end)
            path = s_path
            path_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_s_path_coordinates_{timestamp}.csv' #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\s_path_coordinates{timestamp}.csv'
            vehicle_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_s_vehicle_coordinates_{timestamp}.csv'   #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\s_vehicle_coordinates{timestamp}.csv'
            output_file = rf'/home/bennet/ATRP/Autonomous_Control/Data/Simulation/sim_s_pid_output_{timestamp}.svg'     #'C:\Users\benne\OneDrive\Dokumente\Uni\Bachelor-Arbeit\ATRP\Autonomous_Control\Data\s_output{timestamp}.svg'

    print(path)

    jetson = connect()

    # Initialize Kalman Filter          1 mm² variance, 5 cm² variance, 0.05 cm² initial error 
    #kalman_params = KalmanFilterParams(process_noise=0.001, measurement_noise=0.05, initial_error=0.05)
    #   kf = KalmanFilterGPS(kalman_params)

    # Initialize PID controller 
    pid_params = PIDParams(kp=0.25, ki=0.001, kd=0.75)  #line, test, curve: kp=0.25, ki=0.001, kd=0.5
    pid = PIDController(pid_params)                     #finished: kp=0.25, ki=0.001, kd=0.75

    last_position = start_coordinates
    waypoint_index = 0
    error = 0.0
    error_correction = 0.0

    commands = f"nothing|0.0"

    error_list = []
    error_correction_list = []
    steering_angle_list = []

    previous_gps = None
    # Open CSV files for writing
    with open(path_file, 'w', newline='') as path_file, open(vehicle_file, 'w', newline='') as vehicle_file:
        path_writer = csv.writer(path_file)
        vehicle_writer = csv.writer(vehicle_file)

        # Write headers
        path_writer.writerow(['Latitude', 'Longitude'])
        vehicle_writer.writerow(['Latitude', 'Longitude'])

        # Write path coordinates to CSV file
        for coord in path:
            path_writer.writerow(coord)

        while connected and waypoint_index < len(path) - 1:
            data = jetson.recv(2048).decode("utf8")
            orderedData = extractData(data)
            
            if orderedData is None or keyboard.is_pressed("escape") or keyboard.is_pressed("k"):
                commands = f"escape|0.0"
                jetson.send(bytes(commands, "utf-8"))
                disconnect(jetson)

                plt.figure()
                plt.subplot(3, 1, 1)
                plt.plot(error_list, label='Error')
                plt.axhline(y=0, color='r', linestyle='--')
                plt.xlabel('Time Step')
                plt.ylabel('Error')
                plt.ylim(-4, 4)
                plt.legend()

                plt.subplot(3, 1, 2)
                plt.plot(steering_angle_list, label='Steering Error')
                plt.axhline(y=0, color='r', linestyle='--')
                plt.xlabel('Time Step')
                plt.ylabel('Steering Error')
                plt.ylim(-16.0, 16.0)
                plt.legend()
            
                plt.subplot(3, 1, 3)
                plt.plot(error_correction_list, label='PID Output')
                plt.axhline(y=0, color='r', linestyle='--')
                plt.xlabel('Time Step')
                plt.ylabel('PID Output')
                plt.ylim(-16.0, 16.0)
                plt.legend()
            
                plt.tight_layout()
                plt.savefig(output_file, format='svg')
                plt.show()

                break

            if orderedData is not None:
                current_gps = orderedData["gps"]
                print(current_gps)

                if current_gps[0] > 0.1 and current_gps[1] > 0.1 and (previous_gps is None or current_gps != previous_gps):
                    command, waypoint_index, error, error_correction, last_position, steering_error = commandLoop(pid, path, current_gps, waypoint_index, last_position)
                    steering_angle = error_correction

                    if command == "":
                        disconnect(jetson)
                        break

                    # send commands
                    commands = f"{command}|{steering_angle:.8f}"
                    jetson.send(bytes(commands, "utf8"))

                    # print(current_gps)
                    print("Command: %s | Waypoint: %d | Error: %f | Steering Error: %f | GPS: %f, %f | Steering: %f \r" % (command, waypoint_index, error, steering_error, current_gps[0], current_gps[1], steering_angle))

                    error_list.append(error)
                    error_correction_list.append(error_correction)
                    steering_angle_list.append(steering_error)

                    # save old data
                    previous_gps = current_gps

                    # Write vehicle coordinates to CSV file
                    vehicle_writer.writerow(current_gps)
                else:
                    jetson.send(bytes(commands, "utf-8"))
        

            # controlls the update time for controller and driver
            time.sleep(0.5)    # 0.5 seconds -> twice the time of the gps update rate of 1 Hz

    commands = f"escape|0.0"
    jetson.send(bytes(commands, "utf-8"))

    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(error_list, label='Error')
    plt.axhline(y=0, color='r', linestyle='--')
    plt.xlabel('Time Step')
    plt.ylabel('Error')
    plt.ylim(-4, 4)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(steering_angle_list, label='Steering Error')
    plt.axhline(y=0, color='r', linestyle='--')
    plt.xlabel('Time Step')
    plt.ylabel('Steering Error')
    plt.ylim(-16.0, 16.0)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(error_correction_list, label='PID Output')
    plt.axhline(y=0, color='r', linestyle='--')
    plt.xlabel('Time Step')
    plt.ylabel('PID Output')
    plt.ylim(-16.0, 16.0)
    plt.legend()

    plt.tight_layout()
    plt.savefig(output_file, format='svg')
    plt.show()

    sys.stdout.write("\n")
    print("Exiting...")

if __name__ == "__main__":
    main(sys.argv)