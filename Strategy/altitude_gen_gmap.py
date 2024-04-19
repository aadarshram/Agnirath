import csv
import googlemaps

# Replace 'YOUR_API_KEY' with your actual API key
api_key = 'AIzaSyDKKVK8NtYOa_DDPafADUeISiJR3eIm4ig'

# Function to retrieve elevation data for a batch of coordinates
def get_elevation_data(coordinates):
    gmaps = googlemaps.Client(key=api_key)
    elevation_data = gmaps.elevation(coordinates)
    return elevation_data

# Read the CSV file containing latitude and longitude
csv_filename = 'coordinates.csv'

# Initialize an empty list to store elevation data
elevation_data_list = []

with open(csv_filename, 'r') as csvfile:
    csv_reader = csv.reader(csvfile)
    
    # Skip the header row
    next(csv_reader)
    
    # Initialize a batch list to hold coordinates
    batch = []
    
    for row in csv_reader:
        if len(batch) < 511:
            # Append latitude and longitude as a tuple
            batch.append((float(row[0]), float(row[1])))
        else:
            # Process the batch
            elevation_data = get_elevation_data(batch)
            elevation_data_list.extend(elevation_data)
            
            # Reset the batch for the next set of coordinates
            batch = []

    # Process the last batch (if any)
    if batch:
        elevation_data = get_elevation_data(batch)
        elevation_data_list.extend(elevation_data)

# Update the CSV file with elevation data
output_csv_filename = 'coordinates.csv'
with open(output_csv_filename, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(["latitude","longitude","altitude"])
    for elevation_data_point in elevation_data_list:
        latitude = elevation_data_point['location']['lat']
        longitude = elevation_data_point['location']['lng']
        elevation = elevation_data_point['elevation']
        csv_writer.writerow([latitude, longitude, elevation])

print(f'Elevation data updated in {output_csv_filename}')
