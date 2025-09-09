import pickle
from plotter_utility.classes_ros import *
from plotter_utility import config
from plotter_utility.printing import update
from tqdm import tqdm
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Things we could plot
# Gps, DVL DR, Factor Graph, waypoints,Desired Heading arrows, bathymetry

def run(args):
    update('Starting interpretation', True)

    data_dict = {
        config.INTERPRETED_HEADING: [],
        config.INTERPRETED_IMU: [],
        config.INTERPRETED_BEAM_RANGE: [],
        config.INTERPRETED_BEAM_INTENSITY: [],
        config.INTERPRETED_GPS: [],
        config.INTERPRETED_GPS_ODOM: [],
        config.INTERPRETED_FACTOR: [],
    }

    if args.file == 'null':
        update('ERROR: A valid rosbag file is required when running the interpreter', True)
        quit(2)

    update('Opening rosbag file: ' + args.file)

    try:
        # Set up the rosbag2 reader
        storage_options = rosbag2_py.StorageOptions(uri=args.file, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        # Get the list of topics and types from the rosbag metadata
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        # Iterate through messages in the rosbag
        update('Iterating over rosbag file:', True)
        while reader.has_next():
            (topic, msg, t) = reader.read_next()

            # Deserialize the message
            msg_type = get_message(type_map[topic])
            deserialized_msg = deserialize_message(msg, msg_type)

            # Process messages based on the topic name
            if topic.endswith('/gps_odom'):
                data_dict[config.INTERPRETED_GPS_ODOM].append(ODOM(deserialized_msg))
            elif topic.endswith('/fix'):
                data_dict[config.INTERPRETED_GPS].append(GPS_FIX(deserialized_msg))
            elif topic.endswith('/smoothed_output'):
                data_dict[config.INTERPRETED_FACTOR].append(ODOM(deserialized_msg))
            # elif topic.endswith('/fix'):
            #     data_dict[config.INTERPRETED_GPS].append(GPS_FIX(deserialized_msg))
    except Exception as e:
        update(f"ERROR: Failed to read rosbag: {str(e)}", True)
        quit(2)

    # Store the interpreted data
    update('Storing interpreted data')
    with open(config.INTERPRETER_OUTPUT, 'wb') as fp:
        pickle.dump(data_dict, fp)

    update('Rosbag interpretation complete', True)
