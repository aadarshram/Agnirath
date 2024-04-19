import requests
import csv
import math

# Mapbox API endpoint for directions
api_endpoint = 'https://api.mapbox.com/directions/v5/mapbox/driving-traffic'

# Mapbox API access token
access_token = 'pk.eyJ1IjoiamFheWFudGgiLCJhIjoiY2xpcHVpMHlzMG01MDNmbGI1NTZoNDVpciJ9.GO9DSZT5E9tGdEKg8WAp9Q'

# Start point, waypoints, and endpoint coordinates
start_point = [-12.466345, 130.848252][::-1]  # [longitude, latitude]
waypoints = [[-14.463715, 132.261138][::-1],
             [-16.253637, 133.369386][::-1],
             [-16.679739, 133.412201][::-1],
             [-19.645053, 134.191294][::-1],
             [-21.530901, 133.889022][::-1],
             [-23.699087, 133.877777][::-1],
             [-25.839810, 133.300668][::-1],
             [-25.998358, 133.196110][::-1],
             [-27.302725, 133.622988][::-1],
             [-29.021883, 134.759792][::-1],
             [-30.967757, 135.749018][::-1],
             [-32.471055, 137.739759][::-1]
             ]  # [longitude, latitude]
endpoint = [-34.923484, 138.593293][::-1]  # [longitude, latitude]

# Construct the coordinates string for the waypoints
waypoints_str = ';'.join([f'{coord[0]},{coord[1]}' for coord in waypoints])

# Construct the request URL
request_url = f'{api_endpoint}/{start_point[0]},{start_point[1]};{waypoints_str};{endpoint[0]},{endpoint[1]}'
params = {
    'access_token': access_token,
    'geometries': 'geojson',
    'steps': 'true',
}

# Send the API request
response = requests.get(request_url, params=params)
data = response.json()

# Extract route information
route = data['routes'][0]
legs = route['legs']

# Extract coordinates from each leg of the route
coordinates = []

for leg in legs:
    steps = leg['steps']
    for step in steps:
        geometry = step['geometry']
        coords = geometry['coordinates']
        for i in range(len(coords) - 1):
            lat1, lon1 = coords[i]
            lat2, lon2 = coords[i + 1]
            distance = math.sqrt((lat2 - lat1) ** 2 + (lon2 - lon1) ** 2) * 111.32 * 1000  # Calculate distance in meters
            if distance >= 10:
                num_points = int(distance / 10)
                for j in range(num_points):
                    fraction = j / num_points
                    new_lat = lat1 + fraction * (lat2 - lat1)
                    new_lon = lon1 + fraction * (lon2 - lon1)
                    coordinates.append([new_lon, new_lat])

# Save the data to a CSV file with the switched columns
csv_filename = 'coordinates.csv'

with open(csv_filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['latitude', 'longitude'])  # Header with "Altitude"
    writer.writerows(coordinates)

print(f'Route information saved to {csv_filename}.')
