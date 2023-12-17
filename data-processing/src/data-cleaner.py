import os
import polars as pl

import datetime
from multiprocessing import Pool

input_path = os.path.join(os.path.dirname(__file__), '../data/raw/PrintorOutput/')
output_path = os.path.join(os.path.dirname(__file__), '../data/processed/')

def process_file(file_name):
    print("Processing file: " + file_name)
    if file_name.endswith(".txt"):
        if file_name.startswith("py"):
            with open(input_path + file_name) as file:
                df = pl.DataFrame({'timestamp':[''], 'dist':[''], 'bend':['']})
                for line in file.readlines():
                    line = line.replace("Maximum deflection:", '')
                    if not line.startswith('{'):
                        continue
                    split = line.split(',')

                    timestamp = split[0].split(':')[1]
                    dist = split[1].split(':')[1]
                    bend = split[2].split(':')[1].split('}')[0]

                    df2 = pl.DataFrame({'timestamp':[timestamp], 'dist':[dist], 'bend':[bend]})
                    df = df.vstack(df2)
                df = df.slice(1, df.height)
                output_file_name = file_name.replace('txt', 'csv')
                df.write_csv(output_path + output_file_name)
                        
        elif file_name.startswith("rust"):
            with open(input_path + file_name) as file:
                #create a dataframe with columns date, time, ss.x, ss.y, ss.z, rc.x, rc.y, rc.z
                df = pl.DataFrame({'timestamp':[''], 'ss.x':[''], 'ss.y':[''], 'ss.z':[''], 'rc.x':[''], 'rc.y':[''], 'rc.z':['']})
                for line in file.readlines():
                    if not line.startswith('') or not line.endswith('})\n'):
                        continue
                    split = line.split('[')

                    timestamp = split[1]
                    date, time_str = timestamp.split('T')
                    date = date.split('m')[1]
                    time_str = time_str.split('Z')[0]

                    date_time = datetime.datetime.strptime(date + ' ' + time_str, '%Y-%m-%d %H:%M:%S.%f')
                    timestamp = str(datetime.datetime.timestamp(date_time))

                    data = split[8].replace('\n', '').split(' - ')
                    SensorSample, ResCentering = data[0].replace(' ', ''), data[1].replace(' ', '')

                    ss_x, ss_y, ss_z = \
                        SensorSample.split(':')[1].split(',')[0],\
                        SensorSample.split(':')[2].split(',')[0],\
                        SensorSample.split(':')[3].split(',')[0].split('}')[0]
                    rc_x, rc_y, rc_z = \
                        ResCentering.split(':')[1].split(',')[0],\
                        ResCentering.split(':')[2].split(',')[0],\
                        ResCentering.split(':')[3].split(',')[0].split('}')[0]
                    
                    #add the data to the dataframe
                    df2 = pl.DataFrame({'timestamp':[timestamp], 'ss.x':[ss_x], 'ss.y':[ss_y], 'ss.z':[ss_z], 'rc.x':[rc_x], 'rc.y':[rc_y], 'rc.z':[rc_z]})
                    df = df.vstack(df2)
                #remove the first row of the dataframe which is empty
                df = df.slice(1, df.height)
                #save the dataframe to a csv file with the same name as the input file
                output_file_name = file_name.replace('txt', 'csv')
                df.write_csv(output_path + output_file_name)
                
        else:
            raise Exception("File name does not start with python or rust")
    print("Processed file: " + file_name)
                
if __name__ == '__main__':
    with Pool(8) as p:
        p.map(process_file, os.listdir(input_path))

