# from plotter_processes.archive import csv_interpreter, rosbag_interpreter
from plotter_processes import data_processor, map_generator
import argparse
from plotter_utility import config
from datetime import datetime

def process_data(folder, maptype='sat', latlongaxis=False, process='all', colormap='jet_r', 
                 samplereduction=1, location='online', behavior=False):
    
    # Create an argparse.Namespace object to mimic command-line arguments
    args = argparse.Namespace(
        folder=folder,
        maptype=maptype,
        latlongaxis=latlongaxis,
        process=process,
        colormap=colormap,
        samplereduction=samplereduction,
        location=location,
        behavior=behavior
    )

    if args.samplereduction <= 0:
        raise ValueError('Sample reduction must be an int greater than 0')

    if args.process == 'all':
        data_processor.run(args)
        map_generator.run(args)
    elif args.process == 'dat':
        data_processor.run(args)
    elif args.process == 'map':
        map_generator.run(args)
    else:
        raise ValueError(f"Invalid process option: {args.process}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some maps.')
    parser.add_argument('folder', help='the containing rosbag and bhv', default='null')

    parser.add_argument('-m', '--maptype', choices=['rdm', 'sat', 'non'], default='sat',
                        help="Specify the type of map (default: sat). See the READ.md for more")
    parser.add_argument('-g', '--latlongaxis', action='store_true', help='In mapping, use latitude / logitude coordinates for the axis instead of meters')
    parser.add_argument('-P', '--process', choices=['csv', 'dat', 'map', 'all','rosbag'], default='all', help='Specifiy which of the subprocesses will run')
    parser.add_argument('-c', '--colormap', choices=config.COLORMAP_OPTIONS, default='jet_r', help='The colormap used for the plotting')
    parser.add_argument('-s', '--samplereduction', type=int, default=1, help='The number by which to reduce the data. For example, if n is input, only (1/n)th of the data will be processed and mapped.')
    parser.add_argument('-l', '--location', choices=['ul', 'byu','online'], default='online', help='Specifiy which map location. Defaults to online map query')
    parser.add_argument('-b', '--behavior', action='store_true', help='a .bhv file is in directory containing the waypoint data')

    args = parser.parse_args()

    if (args.samplereduction <= 0):
        print('Sample reduction must be an int greater than 0')

    process = args.process


    if (process == 'all'):
        # rosbag_interpreter.run(args)
        data_processor.run(args)
        map_generator.run(args)
    elif (process == 'dat'):
        data_processor.run(args)
    elif (process == 'map'):
        map_generator.run(args)