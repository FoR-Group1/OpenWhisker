import polars as pl
from sklearn.model_selection import train_test_split

import os


input_path = os.path.join(os.path.dirname(__file__), '../data/processed/combined/combined_xsmall.csv')
output_path = os.path.join(os.path.dirname(__file__), '../data/processed/combined/split')

# Read the CSV file
data = pl.read_csv(input_path)

# Split the dataset into training and testing sets
train_data, test_data = train_test_split(data, test_size=0.2, random_state=42)

# Save the training and testing datasets to separate CSV files
train_data.write_csv(output_path+'/train_data_xsmall.csv')
test_data.write_csv(output_path+'/test_data_xsmall.csv')


