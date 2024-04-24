# import libraries 

import pandas as pd
import requests
from requests.structures import CaseInsensitiveDict
import json
from itertools import pairwise
import numpy as np
# --------------------------------------------------------------------------------------------------------------
# API Config

GEOAPIFY_API_KEY = "bdd42bb219284ccf8141a5e38e7c300d"
GOOGLEMAPS_API_KEY = "AIzaSyDKKVK8NtYOa_DDPafADUeISiJR3eIm4ig"

# --------------------------------------------------------------------------------------------------------------
# Route Data

WayPoints = [
    # lat, long
    [-12.460477, 130.842984],  # Darwin (Espalade) 
    [-13.236992, 131.102707],  # Adelaide river (should reach here exactly after 5 hours of start)
    [-14.502924, 132.361515],  # Control Stop 1  
    [-16.674307, 133.408777],  # Control Stop 2  
    [-18.871373, 134.141186],  # Control Stop 3  
    [-21.531345, 133.889130],  # Control Stop 4   
    [-23.708208, 133.873830],  # Control Stop 5   
    [-29.011015, 134.754905],  # Control Stop 6  
    [-30.969925, 135.748925],  # Control Stop 7 
    [-34.639701, 138.653775],  # Adelaide (Victoria Square)
]

# --------------------------------------------------------------------------------------------------------------
# Coordinate Retrival

def request_coordinates(locs):
    # request API
    url = f"https://api.geoapify.com/v1/routing?waypoints={'|'.join(','.join(map(str, pt)) for pt in locs)}&mode=drive&details=instruction_details,route_details,elevation&apiKey={GEOAPIFY_API_KEY}"
    headers = CaseInsensitiveDict()
    headers["Accept"] = "application/json"
    response = requests.get(url, headers = headers)

    if(response.status_code == 200):
        json_data = json.loads(response.text)

        # with open("testdata.json", "w+") as file:
        #     file.write(json.dumps(json_data))

        coordinates = [pt for set in json_data['features'][0]['geometry']["coordinates"] for pt in set]
        return coordinates
    else:
        print("error in network request:", response.status_code)

# Executor block
print("Routing and finding Coordinates..")
l = request_coordinates(WayPoints)
print("API response recieved.")
print("No of coordinates:", len(l))
print("="*50)

# --------------------------------------------------------------------------------------------------------------
# Utilility functions for Coordinate calculations

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the Haversine distance between two points specified by their latitude and longitude
    using NumPy arrays.
    """
    # Compute differences between coordinates
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1

    # Calculate Haversine distance
    a = np.sin(d_lat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(d_lon / 2) ** 2
    c = 2 * np.arcsin(np.sqrt(a))
    r = 6371  # Radius of the Earth in kilometers
    distance = r * c # convert to km

    return distance * 1000

def intermediate_point(lat1, lon1, lat2, lon2, fraction):
    """
    Calculate the intermediate point at a specified fraction along the great circle path
    between two points specified by their latitude and longitude using NumPy.
    """
    # Calculate angular distance
    delta = 2 * np.arcsin(np.sqrt((np.sin((lat2 - lat1) / 2) ** 2) +
                                np.cos(lat1) * np.cos(lat2) * (np.sin((lon2 - lon1) / 2) ** 2)))

    # Calculate intermediate values
    a = np.sin((1 - fraction) * delta) / np.sin(delta)
    b = np.sin(fraction * delta) / np.sin(delta)
    x = a * np.cos(lat1) * np.cos(lon1) + b * np.cos(lat2) * np.cos(lon2)
    y = a * np.cos(lat1) * np.sin(lon1) + b * np.cos(lat2) * np.sin(lon2)
    z = a * np.sin(lat1) + b * np.sin(lat2)

    # Calculate latitude and longitude of the intermediate point
    lat_intermediate = np.arctan2(z, np.sqrt(x**2 + y**2))
    lon_intermediate = np.arctan2(y, x)

    return [lat_intermediate, lon_intermediate]

# --------------------------------------------------------------------------------------------------------------
# Coordinates Augumentation

print("Augumenting no of coordinates..")
coordinates = []

for (long1, lat1), (long2, lat2) in pairwise(l):
    long1, lat1, long2, lat2 = map(np.radians, (long1, lat1, long2, lat2))

    d = haversine_distance(lat1, long1, lat2, long2)
    n = np.ceil(d/10)

    for i in range(int(n)):
        coordinates.append(
            intermediate_point(lat1, long1, lat2, long2, i/n)
        )
    coordinates.append([lat2, long2])
print("No of coordinates(after augmentation):", len(coordinates))
print("="*50)

coordinates_deg = [[np.degrees(lat), np.degrees(long)] for lat, long in coordinates]

# --------------------------------------------------------------------------------------------------------------
# Elevation retrival

def get_elevation(coords, api_key):
    """
    Get the elevation of a coordinate using the Google Maps Elevation API.

    Parameters:
    latitude (float): Latitude of the coordinate.
    longitude (float): Longitude of the coordinate.
    api_key (str): Your Google Maps API key.

    Returns:
    float: Elevation of the coordinate in meters.
    """
    # Google Maps Elevation API endpoint
    url = f"https://maps.googleapis.com/maps/api/elevation/json?locations={coords}&key={api_key}"

    # Send the request
    response = requests.get(url)
    # print(response.text)
    data = response.json()

    # Extract elevation from response
    if 'results' in data and len(data['results']) > 0:
        return [result['elevation'] for result in data['results']]
    else:
        print("Error encountered\n", response.text)
        return None

form = lambda ls: '|'.join(','.join(map(str, pt)) for pt in ls)

print("Requesting elevation from google with batches of 400 (max allowed batchsize).\nNote it takes about 10min for this section to run..")
elevations_comp = [get_elevation(form(coordinates_deg[i:i+400]), GOOGLEMAPS_API_KEY) for i in range(0, len(coordinates_deg), 400)]
print("All elevation data sucessfully retireved.")
print("="*50)
elevations = [e for set in elevations_comp for e in set]

assert len(elevations) == len(coordinates_deg), "Size mismatch"
# --------------------------------------------------------------------------------------------------------------
# Creating & exporting Coordinate-Elevation dataframe
step_distance = [haversine_distance(*map(np.radians, (lat1, lon1, lat2, lon2))) for (lat1, lon1), (lat2, lon2) in pairwise(coordinates_deg)]
cumulative_distance = np.cumsum(step_distance)/1000
df = pd.DataFrame({
    "Lattitude": [lat for (lat, _) in coordinates_deg],
    "Longitude": [long for (_, long) in coordinates_deg],
    "Elevation": elevations,
    "Cumulative distance (km)": np.insert(cumulative_distance, 0,0)
})
df.to_csv("coordinate_data.csv", index=False)
print("Exported Coordinate Data to 'coordinate_data.csv'")

# --------------------------------------------------------------------------------------------------------------
# Creating & exporting road Step-Slope dataframe

df3 = pd.DataFrame()
df3['StepDistance(m)'] = step_distance
df3['CumulativeDistance(km)'] = cumulative_distance
df3['Slope (deg)'] = np.degrees(
    [np.arctan((df['Elevation'][i] - df['Elevation'][i-1])/df3['StepDistance(m)'][i-1]) for i in range(1, len(coordinates))]
)

df3.to_csv("slope_profile.csv", index=False)
print("Exported Slope Data to 'slope_profile.csv'")
