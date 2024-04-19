import requests
import csv
import time

API_KEY = 'bfe3a81810c8138f263b82181dd7666c'
LATITUDE = 13.0827  # Chennai latitude
LONGITUDE = 80.2707  # Chennai longitude

def get_solar_irradiance():
    url = f'https://api.openweathermap.org/data/2.5/onecall?lat={LATITUDE}&lon={LONGITUDE}&appid={API_KEY}&exclude=current,minutely,hourly,alerts&units=metric'
    response = requests.get(url)
    data = response.json()

    if 'daily' in data:
        return data['daily']
    else:
        print('Error: Unable to fetch solar irradiance data')
        print(data)  # Print the response for troubleshooting purposes
        return None

def save_to_csv(irradiance_data):
    with open('solar_irradiance.csv', 'w', newline='') as csvfile:
        fieldnames = ['datetime', 'irradiance']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for day in irradiance_data:
            timestamp = day['dt']
            date_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))
            irradiance = day['solar_radiation']
            writer.writerow({'datetime': date_time, 'irradiance': irradiance})

def main():
    irradiance_data = get_solar_irradiance()

    if irradiance_data:
        # Fetch data every second for the next 5 days
        for _ in range(5 * 24 * 60 * 60):
            save_to_csv(irradiance_data)
            time.sleep(1)

if __name__ == '__main__':
    main()








