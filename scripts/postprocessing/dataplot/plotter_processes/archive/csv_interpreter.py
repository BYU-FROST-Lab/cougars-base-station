import csv
import pickle
from plotter_utility.classes import *
from plotter_utility import config
from plotter_utility.printing import update
import pickle
from tqdm import tqdm

def run(args):
    update('Starting interpretation', True)

    data_dict = {}
    # open the CSV file for reading
    # with open('Tue_May_02_23_10_32am.csv', 'r') as csvfile:

    if (args.csvname == 'null'):
        update('ERROR: A valid .csv file is required when running the interpreter', True)
        quit(2)

    update('opening csv file: ' + args.csvname)

    with open(args.csvname, 'r') as csvfile:
        reader = csv.reader(csvfile)

        # iterate over each row in the CSV file
        header = next(reader)
        data_dict[config.INTERPRETED_HEADING]           = []
        data_dict[config.INTERPRETED_IMU]               = []
        data_dict[config.INTERPRETED_BEAM_RANGE]        = []
        data_dict[config.INTERPRETED_BEAM_INTENSITY]    = []
        data_dict[config.INTERPRETED_GPS]               = []
        row_num = 0

        update('Iterating over csv file:', True)
        try:
            for row in tqdm(reader):
                row_num += 1
                # print(row_num)
                # check the value of the 2nd column
                if row[1][:3] == 'Lat':
                    data_dict[config.INTERPRETED_HEADING].append(Heading(row))
                elif row[1] == 'IMU':
                    #imu = Imu(row)
                    data_dict[config.INTERPRETED_IMU].append(IMU(row))
                elif row[1] == 'Beam Range':
                    data_dict[config.INTERPRETED_BEAM_RANGE].append(BEAM_RANGE(row))
                elif row[1] == 'Beam Intensity':
                    data_dict[config.INTERPRETED_BEAM_INTENSITY].append(BEAM_INTENSITY(row))
                elif row[1] == 'GPS':
                    data_dict[config.INTERPRETED_GPS].append(GPS(row))
                else:
                    print("Error: unknown row type", row)
        except:
            pass

    # print the stats on how big each dictionary is
    # print('heading', len(data_dict['heading']))
    # print('imu', len(data_dict['imu']))
    # print('beam_range', len(data_dict['beam_range']))
    # print('beam_intensity', len(data_dict['beam_intensity']))

    update('Storing interpreted data')
    with open(config.INTERPRETER_OUTPUT, 'wb') as fp:
        pickle.dump(data_dict, fp)

    update('CSV interpretation complete', True)
