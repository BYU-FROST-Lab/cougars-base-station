import googlemaps
# from googlemaps.maps import static_map
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image  as pltimg
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.patches import Circle
from datetime import datetime
import pickle
from tqdm import tqdm

from plotter_utility import config, cordinatehandling
from plotter_utility.printing import update

from dotenv import load_dotenv
import os

def gmaps_env_key():
    # Load environment variables from .env file
    load_dotenv("plotter_utility/.env")

    # Access the environment variables
    api_key = os.getenv('GMAPS_API_KEY')

    # Now you can use the variables in your code
    return api_key

def plot_waypoints(waypoints, radii, ax):
    x_waypoints = waypoints[:, 0]
    y_waypoints = waypoints[:, 1]

    capture_radius = radii[0]
    slip_radius = radii[1]
    # Plot the waypoints
    # ax.scatter(x_waypoints, y_waypoints, color='black', marker='o', label='Waypoints')

    # Add circles at each waypoint
    i = 1
    for x, y in zip(x_waypoints, y_waypoints):
        # Green inner circle with radius 5
        inner_circle = Circle((x, y), radius=capture_radius, color='green', fill=True, alpha=0.3)
        ax.add_patch(inner_circle)
        
        # Red outer circle with radius 10
        outer_circle = Circle((x, y), radius=slip_radius, color='red', fill=False, linewidth=2)
        ax.add_patch(outer_circle)

        # Number the waypoints
        ax.text(x, y, str(i), color='black', fontsize=12, ha='center', va='center', weight='bold')
        i += 1

def add_colored_line(ax, x, y, z, colormap):
        """Adds a colored line to the axis based on z-values."""
        # Create segments for the line
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        # Normalize z-values and apply a colormap
        norm = Normalize(vmin=np.min(z), vmax=np.max(z))
        lc = LineCollection(segments, cmap=colormap, norm=norm)
        lc.set_array(z)
        lc.set_linewidth(2.5)  # Adjust line thickness
        
        # Add the LineCollection to the axis
        ax.add_collection(lc)
        return lc

def plot_line(ax, array, config_name, colors, use_colormap=False):
    if np.size(array) == 0:
        return
    x_values = array[:,0]
    y_values = array[:,1]
    z_values = array[:,2]

    if use_colormap:
        colormap = plt.get_cmap('viridis')  # You can choose any colormap
        lc = add_colored_line(ax, y_values, x_values, z_values, colormap)
        plt.colorbar(lc, ax=ax, orientation='vertical', label=f'{config_name} Z-Values')
    else:
        color = colors.pop(0) if colors else 'black'
        plt.plot(y_values, x_values, color=color, label=config_name)


def run(args):
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan', 'magenta', 'yellow']

    update('Starting map generation', True)

    # Load in the data
    update("Unpackaging processed data")
    with open(config.PROCESSOR_OUTPUT, 'rb') as fb:
        data_dict = pickle.load(fb)
    centerCords = data_dict[config.PROCESSED_CENTER]
    path = data_dict[config.PROCESSED_PATH]

    # Determin the zoom level and picture radius
    update("Determining zoom level and picture size")
    maxX = np.max(np.abs(path[0]))
    maxY = np.max(np.abs(path[1]))
    maxDistance = maxX if maxX >= maxY else maxY
    maxDistance = 100
    #TODO: Fix the max distance calculation. Not sure how it works
    # print("Center Coords", centerCords)
    zoomLevel, pictureRadius = cordinatehandling.DetermineMinimumZoomLevel(centerCords[0], maxDistance)

    # print("Picture Radius", pictureRadius, "Zoom:", zoomLevel)
    # Determine the style of the map
    if   (args.maptype == 'non'):
        makeMap = False
    elif (args.maptype == 'rdm'):
        makeMap = True
        mapType = "roadmap"
    elif (args.maptype == 'sat'):
        makeMap = True
        mapType = "satellite"

    if args.location == 'ul':
        load_map = config.UTAH_LAKE_MAP
        # pictureRadius = #TODO when loading downloaded map choose the picture radius that is correct
    elif args.location == 'byu':
        #TODO: Script to generate new maps based on ref lat and lon easily
        #TODO: Zoom level with the ref lat and lon
        update("Connecting with Google Maps")
        try:
            gmaps = googlemaps.Client(key = gmaps_env_key())
        except ValueError:
            quit(1)

        # Puts the map at the specified location
        update("Generating Google Map")
        mapAtLocation = gmaps.static_map(size=config.DEFAULT_OUTPUT_IMAGE_SIZE, center=cordinatehandling.CordToString(centerCords), maptype=mapType, zoom=zoomLevel)

        update("Saving Google Map")
        output = open(config.EMPTY_MAP, 'wb')
        for chunk in mapAtLocation:
            if chunk:
                output.write(chunk)
        output.close()
        load_map = config.EMPTY_MAP
        # load_map = config.BYU_MAP

    elif makeMap:
        # Initializing the gmaps object
        update("Connecting with Google Maps")
        try:
            gmaps = googlemaps.Client(key = config.GMAPS_API_KEY)
        except ValueError:
            quit(1)

        # Puts the map at the specified location
        update("Generating Google Map")
        mapAtLocation = gmaps.static_map(size=config.DEFAULT_OUTPUT_IMAGE_SIZE, center=cordinatehandling.CordToString(centerCords), maptype=mapType, zoom=zoomLevel)

        update("Saving Google Map")
        output = open(config.EMPTY_MAP, 'wb')
        for chunk in mapAtLocation:
            if chunk:
                output.write(chunk)
        output.close()
        load_map = config.EMPTY_MAP

    # Create the plot
    fig, ax = plt.subplots()
    if makeMap:
        map = pltimg.imread(load_map)
        x_min = - pictureRadius
        x_max = pictureRadius
        y_min = - pictureRadius
        y_max = pictureRadius
        plt.imshow(map, extent=[x_min, x_max, y_min, y_max,])

    update("Generating scatterplot from path data")
    plot_line(ax, data_dict[config.PROCESSED_PATH], 'GPS Fix', colors)
    plot_line(ax, data_dict[config.PROCESSED_GPS_ODOM], 'GPS Odometry', colors)
    plot_line(ax, data_dict[config.PROCESSED_FACTOR], 'Factor Graph', colors)
    plot_line(ax, data_dict[config.PROCESSED_DR], 'Dead Reckoning', colors)

    # Add legend
    plt.legend()

    update("Configuring scatterplot")
    plt.xlabel('Distance from center (Meters)')
    plt.ylabel('Distance from Center (Meters)')
    
    plt.axis('tight')
    
    plt.ticklabel_format(style='plain', useOffset=False)
    # plt.title('Digital Elevation Model (DEM)')
    plt.tight_layout(rect=(0.01, 0.05, 0.99, 0.99))

    update("Saving scatterplot")
    # prompt input for naming of map 

    update("Map generation complete", True)

    #Plot the waypoints
    if args.behavior:
        waypoints = data_dict[config.PROCESSED_WPT]
        radii = data_dict[config.PROCESSED_RAD]
        plot_waypoints(waypoints, radii, ax)
    
    # Show the plot
    # plt.show()
    plt.savefig("output.png", pad_inches=1)