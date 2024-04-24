# interactive plot using plotly

# import libraries
import plotly.express as px
import pandas as pd
import plotly.graph_objects as go

df1 = pd.read_csv('coordinate_data.csv')
df2 = pd.read_csv('slope_profile.csv')

# ----------------------------------------------------------------------

# elevation vs distance plot

df1 = df1.sort_values(by = "Cumulative distance (km)")
fig = px.line(df1, x = "Cumulative distance (km)", y = "Elevation", title = "Elevation (m) vs Distance (km) plot")
for i in range(600,2401,600):
    fig.add_vline(x = i, line_color = "gray")
fig.show()

# -----------------------------------------------------------------------------------

# slope vs cumulative distance plot

df2 = df2.sort_values(by = "CumulativeDistance(km)")
fig = px.line(df2, x = "CumulativeDistance(km)", y = "Slope (deg)", title = "Slope (deg) vs Cumulative distance (km) plot")
fig.show()

# -----------------------------------------------------------------------------------------

# plot coordinates onto map

WayPoints = [
    # lat, long
    [-12.460477, 130.842984],  # Darwin (Espalade)
    [-13.236992, 131.102707],  # Adelaide river(should reach here exactly aftet 5 hours of start)
    [-14.502924, 132.361515],  # Control Stop 1 
    [-16.674307, 133.408777],  # Control Stop 2
    [-18.871373, 134.141186],  # Control Stop 3 
    [-21.531345, 133.889130],  # Control Stop 4 
    [-23.708208, 133.873830],  # Control Stop 5 
    [-29.011015, 134.754905],  # Control Stop 6 
    [-30.969925, 135.748925],  # Control Stop 7
    [-34.639701, 138.653775],  # Adelaide (Victoria Square)
]

df_temp = pd.DataFrame()
df_temp['lat'] = df1['Lattitude']
df_temp['long'] = df1['Longitude']
df_temp['elevation'] = df1['Elevation']
waypoints_df = pd.DataFrame(WayPoints, columns = ['lat', 'lon'])

trace_route = go.Scattermapbox(
    lat=df_temp['lat'],
    lon=df_temp['long'],
    mode='lines',
    hoverinfo='text',
    hovertext=df_temp['elevation'].astype(str),
    line=dict(color='blue', width=2),
)
trace_waypoints = go.Scattermapbox(
    lat=waypoints_df['lat'],
    lon=waypoints_df['lon'],
    mode='markers',
    marker=dict(size=10, color='red'),
)
fig = go.Figure(data=[trace_route, trace_waypoints])
# Update layout to use OpenStreetMap style
fig.update_layout(mapbox_style="open-street-map")
# Set margin to 0 for full map display
fig.update_layout(margin={"r": 0, "t": 0, "l": 0, "b": 0})
fig.show()

# -----------------------------------------------------------------------------------