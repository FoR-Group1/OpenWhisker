import os
import polars as pl
from tqdm import tqdm


input_path = os.path.join(os.path.dirname(__file__), '../data/processed')
output_path = os.path.join(os.path.dirname(__file__), '../data/processed/combined')

python_files = [f for f in os.listdir(input_path) if f.startswith("python")]
rust_files = [f for f in os.listdir(input_path) if f.startswith("rust")]

nb_rows = 0
include_header1, include_header2 = True, True
for py_file in python_files:
    py_df = pl.read_csv(input_path + '/' + py_file)
    total_rows = py_df.height
    with tqdm(total=total_rows, desc="Processing file: " + py_file) as pbar:
        for py_dp in py_df.iter_rows():
            pbar.update(1)
            timestamp = py_dp[0]
            for rs_file in rust_files:
                rs_df = pl.read_csv(input_path + '/' + rs_file)
                for rs_dp in rs_df.iter_rows():
                    if round(timestamp, 2) != round(rs_dp[0], 2):
                        df2 = pl.DataFrame({'timestamp':[str(timestamp)], 'ss.x':[str(rs_dp[1])], 'ss.y':[str(rs_dp[2])], 'ss.z':[str(rs_dp[3])]})
                        with open(output_path + f'/combined_unlabelled.csv', 'ab') as f:
                            df2.write_csv(f, include_header=include_header1)
                        include_header1 = False
                        continue

                    df2 = pl.DataFrame({'timestamp':[str(timestamp)], 'ss.x':[str(rs_dp[1])], 'ss.y':[str(rs_dp[2])], 'ss.z':[str(rs_dp[3])], 'Dist':[str(round(py_dp[1], 4))], 'Bend':[str(round(py_dp[2], 4))]})
                    with open(output_path + f'/combined_labelled.csv', 'ab') as f:
                        df2.write_csv(f, include_header=include_header2)
                    include_header2 = False

