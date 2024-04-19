import pandas as pd
import numpy as np

# Replace 'coordinates.csv' with the actual path to your CSV file








# Drop the last row since there's no consecutive point for it
df = df[:-1]

# Display the new DataFrame with distance and slope columns
print(df)
