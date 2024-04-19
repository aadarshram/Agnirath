import csv
import requests

def get_altitude(latitude, longitude):
    url = f"https://api.open-meteo.com/v1/elevation?latitude={latitude}&longitude={longitude}"
    
    try:
        response = requests.get(url)
        response.raise_for_status()  # Raise an exception for HTTP errors
        data = response.json()

        if 'elevation' in data and len(data['elevation']) > 0:
            return data['elevation'][0]
        else:
            print("No altitude data found for this coordinate.")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Error making the API request: {e}")
        return None
    except Exception as e:
        print(f"Error: {e}")
        return None

input_file = 'coordinates_3.csv'  # Update with your existing CSV file

# Open the CSV file in read and write mode
with open(input_file, 'r+', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    fieldnames = reader.fieldnames

    # Create a temporary list to store the updated data
    updated_rows = []

    total_rows = sum(1 for _ in reader)  # Count the total rows
    csvfile.seek(0)  # Reset the file pointer to read again
    next(reader)  # Skip the header

    processed_rows = 0  # Counter for processed rokkws

    for row in reader:
        if row['altitude'] == 'N/A':
            latitude, longitude = float(row['latitude']), float(row['longitude'])
            altitude = get_altitude(latitude, longitude)
            if altitude is not None:
                row['altitude'] = altitude
        updated_rows.append(row)
        processed_rows += 1
        print(f"Processed {processed_rows}/{total_rows}: {row}")

    # Move the file pointer to the beginning to overwrite the existing content
    csvfile.seek(0)
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    # Write the updated data back to the CSV file
    writer.writerows(updated_rows)

    # Truncate any remaining content if the updated data is shorter
    csvfile.truncate()

print("Updated 'altitude' column in the existing CSV file.")