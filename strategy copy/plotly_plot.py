import plotly.express as px
import pandas as pd

# Load the CSV file
input_file = 'coordinates.csv'
df = pd.read_csv(input_file)

# Create a line graph
fig = px.line_3d(df, x='longitude', y='latitude', z='altitude', title='Altitude vs. Latitude and Longitude')

# Customize the graph layout
fig.update_layout(
    scene=dict(
        xaxis_title='Latitude',
        yaxis_title='Longitude',
        zaxis_title='Altitude',
        aspectmode='manual',  # Ensure a consistent aspect ratio
        aspectratio=dict(x=1, y=1, z=1),  # Set the aspect ratio to 1:1:1
    )
)

# Show the graph
fig.show()
