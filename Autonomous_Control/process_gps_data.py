import csv
import matplotlib.pyplot as plt
import math

def gps_to_meters(lat, lon, start_lat, start_lon):
    """Convert GPS coordinates to local ENU coordinates."""
    earth_radius = 6378137  # Radius of Earth in meters
    dlat = math.radians(lat - start_lat)
    dlon = math.radians(lon - start_lon)
    mean_lat = math.radians((lat + start_lat) / 2.0)
    
    x = dlon * earth_radius * math.cos(mean_lat)  # East
    y = dlat * earth_radius  # North

    return x, y


def read_csv(file_path):
    """Read GPS coordinates from a CSV file."""
    coordinates = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        for row in reader:
            lat, lon = map(float, row)
            coordinates.append((lat, lon))
    return coordinates

def convert_coordinates(coordinates, start_lat, start_lon):
    """Convert a list of GPS coordinates to meter coordinates."""
    meter_coordinates = [gps_to_meters(lat, lon, start_lat, start_lon) for lat, lon in coordinates]
    return meter_coordinates

def plot_coordinates(path_coords, vehicle_coords, output_file):
    """Plot path and vehicle coordinates and save as an SVG file."""
    path_x, path_y = zip(*path_coords)
    vehicle_x, vehicle_y = zip(*vehicle_coords)

    plt.figure()
    plt.plot(path_x, path_y, label='Path')
    plt.plot(vehicle_x, vehicle_y, label='Vehicle', linestyle='--')
    plt.xlabel('East (meters)')
    plt.ylabel('North (meters)')
    plt.legend()
    plt.title('Path vs Vehicle GPS Coordinates')
    plt.savefig(output_file, format='svg')
    plt.show()

def main():
    path_file = '/home/bennet/ATRP/Autonomous_Control/curve_path_coordinates.csv'
    vehicle_file = '/home/bennet/ATRP/Autonomous_Control/curve_vehicle_coordinates.csv'
    output_file = '/home/bennet/ATRP/Autonomous_Control/gps_comparison.svg'

    # Read GPS coordinates from CSV files
    path_coords = read_csv(path_file)
    vehicle_coords = read_csv(vehicle_file)

    # Use the first coordinate as the reference point for conversion
    start_lat, start_lon = path_coords[0]

    # Convert GPS coordinates to meter coordinates
    path_meter_coords = convert_coordinates(path_coords, start_lat, start_lon)
    vehicle_meter_coords = convert_coordinates(vehicle_coords, start_lat, start_lon)

    # Plot and save the coordinates
    plot_coordinates(path_meter_coords, vehicle_meter_coords, output_file)

if __name__ == "__main__":
    main()