#----------------#
# OUTPUT OPTIONS #
#----------------#

# These variables can be changed if you want the program output different data

# Google maps allows a zoom level between 0 and 21, 0 seeing the global and 21 inspecting individual blades of grass  (that's an 
# overexageration, to be clear). This is the default value that the script will use for map generation.  It will check to see if 
# all the points are within the the generated map and adjust until all points are within range.
DEFAULT_ZOOM = 19

# The google maps output size.  The photo will be square.
DEFAULT_OUTPUT_IMAGE_SIZE = 1280

# The base elevation used (IE what is the elevation of the water)
DEFAULT_ELEVATION = 0

# A list of colormaps
COLORMAP_OPTIONS = ['jet_r', 'ocean_r', 'copper', 'inferno_r']

#------------------#
# OUTLIER HANDLERS #
#------------------#

# These variables determine the values by which outliers will be judged

# Outlier data to be removed from the sonar scans. 
SONAR_EDGE_OUTLIER = 40 # the sonar has values between 0-240.  0-40 and 200-240 are the edge cases and not as reliable for scans.

# The smallest value we want the beam range to give.
BEAM_RANGE_MINIMUM = 0.1

#--------------------#
# HARDWARE VARIABLES #
#--------------------#

# These variables will change if the hardware on the WAM-V is changed

# Data from the Sonce sensor
SONCE_SWATH_WIDTH = 120
SONCE_INIT_BEAM_ANGLE = -60 # The beam goes from -60 degrees to 60 degrees, 0 being straight down
SONAR_VALUES_MIN = 0
SONAR_VALUES_MAX = 240

#--------------------#
# INTERNAL VARIABLES #
#--------------------#

# These variables are used by the interal function of the processes.

# The pkl file outputted by csv_interpreter
INTERPRETER_OUTPUT = "plotter_processes/interpreted_data.pkl"

# The pkl file outputted by data_processor
PROCESSOR_OUTPUT = "plotter_processes/processed_data.pkl"

# The png file outputted by map_generator
GENERATOR_OUTPUT = "output.png"

# The empty google map used by the map_generator
EMPTY_MAP = "empty_map.png"
UTAH_LAKE_MAP = "maps/ul_map.png"
BYU_MAP = "maps/byu_map.png"

# Dict items output by the interpreter
INTERPRETED_HEADING         = 'heading'
INTERPRETED_IMU             = 'imu'
INTERPRETED_BEAM_RANGE      = 'beam_range'
INTERPRETED_BEAM_INTENSITY  = 'beam_intensity'
INTERPRETED_GPS             = 'gps'
INTERPRETED_GPS_ODOM        = 'gps_odom'
INTERPRETED_FACTOR          = 'factor'
DEAD_RECKON                 = 'dr'

# Dict items output by the processor
PROCESSED_CENTER        = 'center'
PROCESSED_TIME_START    = 'start_time'
PROCESSED_TIME_DIFF     = 'time_diff'
PROCESSED_BATHYMETRY    = 'bathymetry'
PROCESSED_PATH          = 'GPS'
PROCESSED_GPS_ODOM      = 'odom'
PROCESSED_FACTOR        = 'factor'
PROCESSED_WPT           = 'waypoint'
PROCESSED_RAD           = 'radii'
PROCESSED_DR            = 'dead_reckon'

#--------------------#
# CONSTANT VARIABLES #
#--------------------#

# None of these should ever have to be changed, but here they are regardless.

# The circumfrence and radius of the Earth in meters.
EARTH_CIRCUMFRENCE  = 40075000
EARTH_RADIUS        = 6371000

# Meters per a degree of latitude.
METERS_PER_DEGREE_LAT = EARTH_CIRCUMFRENCE / 360

# Google maps meters per pixel when the zoom level is 0.
MAXIMUM_METERS_PER_PIXEL = 156543.03392

# Time
SECONDS_PER_MINUTE = 60
