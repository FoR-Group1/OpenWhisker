import os
import pickle
import polars as pl
import numpy as np
from tqdm import tqdm

input_path = os.path.join(os.path.dirname(__file__), '../data/processed/combined')
unlabelled_path = os.path.join(input_path, 'combined_unlabelled.csv')

model_path = os.path.join(os.path.dirname(__file__), '../models')
model_file = os.path.join(model_path, 'model_out.pkl')

output_path = os.path.join(os.path.dirname(__file__), '../data/processed/combined')
labelled_path = os.path.join(output_path, 'labelled_by_model.csv')

# Load the models
model = None
with open(model_file, 'rb') as f:
    model = pickle.load(f)


df_unlabelled = pl.read_csv(unlabelled_path)
data = df_unlabelled.to_numpy()
X = data[:, 1:]
pred = model.predict(X)

# print(pred[380000:380100])

for i in tqdm(range(len(pred))):
    with open(labelled_path, 'ab') as f:
        dist = str(round(pred[i, 0], 4)) if pred[i, 0] > 0 else 0.0
        bend = str(round(pred[i, 1], 4)) if pred[i, 1] > 0 else 0.0
        df2 = pl.DataFrame({'timestamp':[str(data[i, 0])], 'ss.x':[str(data[i, 1])], 'ss.y':[str(data[i, 2])], 'ss.z':[str(data[i, 3])], 'Dist':[dist], 'Bend':[bend]})
        df2.write_csv(f, include_header=(i == 0))
