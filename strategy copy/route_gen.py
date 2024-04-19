import pandas as pd

# Read the CSV file into a DataFrame
df = pd.read_csv('route.csv')  # Replace 'output.csv' with the actual CSV file path

# Convert the DataFrame to a nested list
nested_list = df.values.tolist()

# Print the nested list (optional)
print(nested_list)