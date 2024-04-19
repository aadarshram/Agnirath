from solcast import forecast

res = forecast.radiation_and_weather(
    hours = '10',
    api_key="9iiCFl3djay3KFYH7nWohFtkQ5lslYZE",
    latitude=-34.92437999999991,
    longitude=138.5848899999999,
    output_parameters=['air_temp', 'ghi', 'clearsky_ghi', 'ghi10']
)

a = res.to_pandas()
b = a.head(-1)
print(b, a.size)
