from sklearn.linear_model import LinearRegression
import pickle
import numpy as np
import polars as pl
import matplotlib.pyplot as plt
import os

class RegressionModel():

    def __init__(self, data_path, model=None):        
        if model is None:
            self.model = LinearRegression()
        else:
            self.model = model

        # X, y_dist, y_bend = self.load_data(data_path)
        X, y = self.load_data(data_path)
        path = '../models/'

        # model_dist = self.fit(X, y_dist)
        # model_bend = self.fit(X, y_bend)
        model_out = self.fit(X, y)

        # file_dist, file_bend, i = 'model_dist.pkl', 'model_bend.pkl', 1
        # while os.path.exists(path + file_dist):
        #     file_dist = 'model_dist_' + str(i) + '.pkl'
        #     file_bend = 'model_bend_' + str(i) + '.pkl'
        #     i += 1
        # self.save_model(model_dist, path + file_dist)
        # self.save_model(model_bend, path + file_bend)

        file_out, i = 'model_out.pkl', 1
        while os.path.exists(path + file_out):
            file_out = 'model_out_' + str(i) + '.pkl'
            i += 1
        self.save_model(model_out, path + file_out)


    def load_data(self, path):
        # Load the data
        df = pl.read_csv(path)
        # Convert the data to numpy array
        data = df.to_numpy()
        # Split the data into X and y
        X = data[:, 1:-2]
        # y_dist = data[:, -1]
        # y_bend = data[:, -2]
        y = data[:, -2:]

        # return X, y_dist, y_bend
        return X, y
        

    def fit(self, X, y):
        return self.model.fit(X, y)

    def predict(self, X):
        return self.model.predict(X)
    
    def score(self, X, y):
        return self.model.score(X, y)

    def save_model(self, model, path):
        # os.makedirs(os.path.dirname(path))
        with open(path, 'wb') as f:
            pickle.dump(model, f)

def main():
    model = RegressionModel('../data/processed/combined/labelled_by_model.csv')

    # model_bend = pickle.load(open('../models/model_bend.pkl', 'rb'))
    # model_dist = pickle.load(open('../models/model_dist.pkl', 'rb'))
    
    # data = pl.read_csv('../data/processed/combined/split/test_data_no_rc.csv')
    # data = data.to_numpy()
    # X, y_dist, y_bend = data[:, 1:-2], data[:, -2], data[:, -1]
    # pred_dist = model_dist.predict(X)
    # output = np.hstack(([[round(x, 2)] for x in pred_dist], [[y] for y in y_dist]))
    # output = np.hstack((output, [[abs(int(x) - int(y))] for x, y in zip(pred_dist, y_dist)]))
    # print(output)

    # pred_bend = model_bend.predict(X)
    # output = np.hstack(([[round(x, 2)] for x in model_bend.predict(X)], [[y] for y in y_bend]))
    # output = np.hstack((output, [[abs(int(x) - int(y))] for x, y in zip(model_bend.predict(X), y_bend)]))
    # print(output)
    # # print("y_dist:", y_dist)

    # print("dist score:", model_dist.score(X, y_dist))
    # print("bend score:", model_bend.score(X, y_bend))

    model_pkl = pickle.load(open('../models/model_out_2.pkl', 'rb'))

    data = pl.read_csv('../data/processed/combined/combined_labelled.csv')
    data = data.to_numpy()
    X, y = data[:, 1:-2], data[:, -2:]
    pred = model_pkl.predict(X)
    # output = np.hstack(([[round(x, 2), round(y, 2)] for x, y in model_pkl.predict(X)], [[y] for y in y]))
    # output = np.hstack((output, [[abs(int(x) - int(y))] for x, y in zip(model_pkl.predict(X), y)]))
    print(pred[15000:15100, 0])
    print(y[15000:15100, 0])

    print(pred[15000:15100, 1])
    print(y[15000:15100, 1])

    print("score:", model_pkl.score(X, y))
    

if __name__ == '__main__':
    main()

        
    