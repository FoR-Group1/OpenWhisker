import os
import polars as pl
from tqdm import tqdm 

# Define the directory paths
input_path = os.path.join(os.path.dirname(__file__), '../data/processed')
output_path = os.path.join(os.path.dirname(__file__), '../data/processed/combined')

# Get the list of rust and python files
python_files = [f for f in os.listdir(input_path) if f.startswith("python")]
rust_files = [f for f in os.listdir(input_path) if f.startswith("rust")]

include_header1, include_header2 = True, True
# for each rust file
for rs_file in rust_files:
    rs_df = pl.read_csv(input_path + '/' + rs_file)
    total_rows = rs_df.height
    with tqdm(total=total_rows, desc="Processing file: " + rs_file) as pbar:
        # for each datapoint in the rust file
        for rs_dp in rs_df.iter_rows():
            timestamp_rs = round(rs_dp[0], 1)
            ss_x, ss_y, ss_z = str(rs_dp[1]), str(rs_dp[2]), str(rs_dp[3])
            rc_x, rc_y, rc_z = str(rs_dp[4]), str(rs_dp[5]), str(rs_dp[6])
            if rc_x == '0.0' and rc_y == '0.0' and rc_z == '0.0':
                df2 = pl.DataFrame({'timestamp':[str(rs_dp[0])], 'ss.x':[ss_x], 'ss.y':[ss_y], 'ss.z':[ss_z], 'Dist':['0.0'], 'Bend':['0.0']})
                with open(output_path + f'/combined_labelled.csv', 'ab') as f:
                    df2.write_csv(f, include_header=include_header2)
                include_header2 = False
                pbar.update(1)
                continue
            # for each python file
            for py_file in python_files:
                found = False
                py_df = pl.read_csv(input_path + '/' + py_file)
                # for each datapoint in the python file
                for py_dp in py_df.iter_rows():
                    timestamp_py = round(py_dp[0], 1)
                    # if the timestamp of the python file is less than the timestamp of the rust file, break
                    if timestamp_py == timestamp_rs:
                        df2 = pl.DataFrame({'timestamp':[str(rs_dp[0])], 'ss.x':[ss_x], 'ss.y':[ss_y], 'ss.z':[ss_z], 'Dist':[str(round(py_dp[1], 4))], 'Bend':[str(round(py_dp[2], 4))]})
                        with open(output_path + f'/combined_labelled.csv', 'ab') as f:
                            df2.write_csv(f, include_header=include_header2)
                        include_header2 = False
                        found = True
                        break
            # if there are no matching timestamps, write the datapoint to the unlabelled file
            if not found:
                df2 = pl.DataFrame({'timestamp':[str(rs_dp[0])], 'ss.x':[ss_x], 'ss.y':[ss_y], 'ss.z':[ss_z]})
                with open(output_path + f'/combined_unlabelled.csv', 'ab') as f:
                    df2.write_csv(f, include_header=include_header1)
                include_header1 = False
            pbar.update(1)
