import os
import polars as pl
from tqdm import tqdm 
# import argparse

# parser = argparse.ArgumentParser(description='Combine the csv files')
# parser.add_argument('--rust', dest='rust')
# args = parser.parse_args()

# Define the directory paths
input_path = os.path.join(os.path.dirname(__file__), '../data/processed')
output_path = os.path.join(os.path.dirname(__file__), '../data/processed/combined')

# Get the list of rust and python files
python_files = [f for f in os.listdir(input_path) if f.startswith("python")]
rust_files = [f for f in os.listdir(input_path) if f.startswith("rust")]

# rs_file = args.rust
for j, rs_file in enumerate(rust_files):
    rs_df = pl.read_csv(input_path + '/' + rs_file)
    total_rows = rs_df.height
# Iterate over the rows of the rust file
    with tqdm(total=total_rows, desc="Processing file: " + rs_file) as pbar:
        for i, rs_dp in enumerate(rs_df.iter_rows()):
            timestamp = rs_dp[0]
            dist, bend = None, None
            found = False
            # Iterate over the python file
            for py_file in python_files:
                py_df = pl.read_csv(input_path + '/' + py_file)
                # Iterate over the rows of the python file
                for j, py_dp in enumerate(py_df.iter_rows()):
                    # find the row with the timestamp just before the current timestamp
                    if timestamp >= py_dp[0]:
                        continue
                    dist, bend = py_df.item(j-1, 1), py_df.item(j-1, 2)
                    found = True
                    break
                if found:
                    break
            # If the timestamp is not found in the python file, skip the row
            if dist is None or bend is None:
                print(f'Timestamp ({timestamp}) not found in python file')
                continue

            # Write the row to the output file
            df2 = pl.DataFrame({'timestamp':[str(timestamp)], 'ss.x':[str(rs_dp[1])], 'ss.y':[str(rs_dp[2])], 'ss.z':[str(rs_dp[3])], 'Dist':[str(round(dist, 4))], 'Bend':[str(round(bend, 4))]})
            with open(output_path + f'/combined_no_rc.csv', 'ab') as f:
                df2.write_csv(f, include_header=(i == 0 and j == 0))
            pbar.update(1)
