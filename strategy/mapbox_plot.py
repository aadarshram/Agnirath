import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

# Replace with your Mapbox access token
mapbox_token = "pk.eyJ1IjoiamFheWFudGgiLCJhIjoiY2xpcHVpMHlzMG01MDNmbGI1NTZoNDVpciJ9.GO9DSZT5E9tGdEKg8WAp9Q"

# Read CSV file with latitude, longitude, and altitude
df = pd.read_csv("coordinates.csv")

# Create a scatter map plot
fig = px.scatter_mapbox(
    df,
    lat="latitude",
    lon="longitude",
    hover_name="altitude",
    zoom=10,  # Adjust the initial zoom level as needed
)

# Create a line plot to indicate the route
fig.add_trace(
    go.Scattermapbox(
        mode="lines+markers",
        lat=df["latitude"],
        lon=df["longitude"],
        marker=dict(size=8),
        line=dict(width=2, color="blue"),  # Adjust line color and width as needed
        text=df["altitude"],
        hoverinfo="text",
        name="Route",
    )
)

# Add arrows at intervals to indicate direction
arrow_interval = 10  # Adjust the interval as needed (e.g., every 10 data points)
for i in range(0, len(df), arrow_interval):
    arrow_lat = df.iloc[i]["latitude"]
    arrow_lon = df.iloc[i]["longitude"]
    
    fig.add_annotation(
        go.layout.Annotation(
            x=arrow_lon,
            y=arrow_lat,
            ax=10,  # Arrowhead length
            ay=0,
            arrowhead=2,  # Arrowhead style
            showarrow=True,
        )
    )

# Update the map layout using Mapbox
fig.update_layout(
    mapbox_style="streets",  # Choose your desired map style
    mapbox_accesstoken=mapbox_token,
)

# Show the map in a web browser or save it as an HTML file
fig.show()
