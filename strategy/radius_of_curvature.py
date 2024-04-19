import numpy as np
import pandas as pd

# CSV file containing latitude and longitude data
#/home/kailash/Workings/Agnirath_LVS_Strategy/Strategy/Strategy_V6
df1 = pd.read_csv("/home/kailash/Workings/Agnirath_LVS_Strategy/Strategy/Strategy_V6/coordinates.csv", delimiter=',')
#csv_file = 'coordinates.csv'

# Read the CSV file into a Pandas DataFrame
#df1 = pd.read_csv(csv_file)


# Convert latitude and longitude to float and calculate distance
def haversine(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude to radians
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    c = 2 * np.arcsin(np.sqrt(a))
    r = 6371  # Radius of Earth in kilometers
    return c * r * 1000


def r_c(lat1, long1, lat2, long2, lat3, long3):
    a = haversine(lat1, long1, lat2, long2)
    b = haversine(lat2, long2, lat3, long3)
    c = haversine(lat3, long3, lat1, long1)
    num = a*b*c
    den = (np.sqrt(abs((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c))))+0.0001
    r = num/den
    return r

# Convert latitude and longitude columns to NumPy arrays
latitudes = df1['latitude'].values
longitudes = df1['longitude'].values


# Calculate radius of curvatures
radius_of_curvatures = r_c(latitudes[:-2], longitudes[:-2], latitudes[1:-1], longitudes[1:-1], latitudes[2:], longitudes[2:])

# Add the radii to the DataFrame
df1['distance'] = [np.nan] + [np.nan] + list(radius_of_curvatures)

