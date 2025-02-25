import numpy as np
import matplotlib.pyplot as plt
import math

# Define the start, midpoint, and end coordinates
s_start = [53.834180, 10.697603]
s_mid = [53.834253, 10.697881]
s_end = [53.834326, 10.698159]

def create_s_shape_path(start, mid, end, num_points=20):
    """Create an S-shaped path using two quadratic Bézier curves."""
    t_values = np.linspace(0, 1, num_points // 2)
    
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
    s_path = [[53.834159, 10.697536]]
    s_path.extend(waypoints)
    return s_path

def gps_to_meters(lat, lon, start_lat, start_lon):
    """Convert GPS coordinates to local ENU coordinates."""
    earth_radius = 6378137  # Radius of Earth in meters
    dlat = math.radians(lat - start_lat)
    dlon = math.radians(lon - start_lon)
    mean_lat = math.radians((lat + start_lat) / 2.0)
    
    x = dlon * earth_radius * math.cos(mean_lat)  # East
    y = dlat * earth_radius  # North
    return x, y

# Generate the S-shaped path
s_shape_path = create_s_shape_path(s_start, s_mid, s_end)

# Convert GPS coordinates to meters
start_lat, start_lon = s_start
path_in_meters = [gps_to_meters(lat, lon, start_lat, start_lon) for lat, lon in s_shape_path]

# Plot the path for visualization in meters
x_coords, y_coords = zip(*path_in_meters)
plt.plot(x_coords, y_coords, marker='o', label='S-shaped Path in Meters')
plt.scatter([0], [0], color='red', label='Start Point')
plt.xlabel('East (meters)')
plt.ylabel('North (meters)')
plt.legend()
plt.title('S-shaped Path in Meters')
plt.show()

# Add the generated waypoints to the curve_path
global curve_path
curve_path = s_shape_path