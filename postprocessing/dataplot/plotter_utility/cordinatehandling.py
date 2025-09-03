import numpy as np
from plotter_utility import config
import math

# Finds the center of an array of coordinates
# Input: two lists of latitudes and longitudes
# Output: two floats, the latCenter and the longCenter
def FindCenter(latList, longList):
    """
    Computes the median center of two lists of latitude and longitude coordinates.

    Args:
        latList (list or np.ndarray): List or array of latitude values.
        longList (list or np.ndarray): List or array of longitude values.

    Returns:
        tuple: Median latitude and longitude as floats.
    """
    # Convert to numpy arrays if they aren't already
    latList = np.array(latList)
    longList = np.array(longList)

    # Filter out NaN values
    cleaned_lat = latList[~np.isnan(latList)]
    cleaned_long = longList[~np.isnan(longList)]

    # Compute the median of cleaned data
    latCenter = np.median(cleaned_lat)
    longCenter = np.median(cleaned_long)

    return latCenter, longCenter

# Turns a cordinate set into the format read by the googlemaps module
# Input:  a np array of [X/lat value, Y average value,...]
# Output: the string "X,Y"
def CordToString(cordSet):
    return str(cordSet[0]) + ',' + str(cordSet[1])

# Turns a cordinate array into a multi-cordinate formate read by the googlemaps module
# Input:  a 2d np array, column 0 being x/lat values, column 2 being y/long values
# Output: the string "X1,Y1|X2,Y2|...|Xn,Yn"
def CordinateArrayToString(cordArray):
    output = ""
    for CordRow in cordArray:
        output += CordToString(CordRow) + "|"
    output.rstrip('|') # removes the final |
    return output

# Calculates how many meters will be represented in a pixel. The formula was provided on a google developer forum by google
# maps developer Chris Broadfoot: https://groups.google.com/g/google-maps-js-api-v3/c/hDRO4oHVSeM.  I tested it on concrete paths
# outside of the MARB, and the predicted value was correct within 30 centemeters of the original, a margin of error I deem acceptable.
# Input:  the latitude at the center of the image, the zoomLevel on a scale of 0-21
# Output: a float with the Meters per Pixel value
def CalculateMetersPerPixel(centerLatitude, zoomLevel):
    return (config.MAXIMUM_METERS_PER_PIXEL * np.cos(centerLatitude * np.pi / 180)) / np.power(2, zoomLevel)

# Calculates the correct zoom level given the center and the max distance.
# Works by finding the amount of meters from the center to the axis then checking if the farthest distance is within that range.
# Input:  The center latitude and the farthest distance point from the center
# Output: The smallest zoom level that will contain all the points and the picture's radius
def DetermineMinimumZoomLevel(centerLatitude, maxDistance):
    pictureRadius = 0
    currZoomlevel = config.DEFAULT_ZOOM + 1
    while (pictureRadius < maxDistance):
        currZoomlevel -= 1
        pictureRadius = CalculateMetersPerPixel(centerLatitude, currZoomlevel) * config.DEFAULT_OUTPUT_IMAGE_SIZE / 4
    
    # # This next part is a personal preference thing, I think it looks better slightly zoomed out
    # currZoomlevel -=1
    # pictureRadius = CalculateMetersPerPixel(centerLatitude, currZoomlevel) * config.DEFAULT_OUTPUT_IMAGE_SIZE / 4
    
    return currZoomlevel, pictureRadius

# Caclulates the difference in degrees of Latitude and Logitude
# Input:  Two np arrays of [latitude, longitude]
# Output: A np array of [latitude, longitude] representing the difference
def CordinateDifference(first, second):
    return np.array([(second[0] - first[0]), (second[1] - first[1])])
    

# Due to the spherical and non-euclidian nature of the mercator cordinate system, meters per a latitude degree is constant
# while meters per longitude degree depends on the latitude in question.  This function calculates that ratio.
# Note: The earth is not a perfect sphere; it bulges at the middle, so this is an estimate.
# This is an interal function, shouldn't be needed elsewhere.
# Input:  the latitude where the ratio should take place
# Output: None. However, the metersPerLongitudeDegree variable will be set
metersPerLongitudeDegree = -1
def __CalculateMetersPerLongitudeDegree(latitude):
    latitudeRad                 = np.radians(latitude)   # convert to rad since np uses radians
    circumfrenceAtLat           = config.EARTH_CIRCUMFRENCE * np.cos(latitudeRad)
    global metersPerLongitudeDegree
    metersPerLongitudeDegree    = circumfrenceAtLat / 360 # number of degrees in a circle, who knew!
    return

# Converts a Latitude/Longitude degree coordinate set to a meter-based cordinate set compared to a center
# Input:  Two np arrays of [latitude, longitude], the first being the center (origin) and the second being the point
# Output: A np array of [x, y] with the cordinates of the point compared to the center in meters
def LatLangToMeters(center, point):
    global metersPerLongitudeDegree
    if (metersPerLongitudeDegree == -1): # make sure that this has been calculated
        __CalculateMetersPerLongitudeDegree(center[0])

    difference = CordinateDifference(center, point)
    meterCordinates = np.array([(difference[0] * config.METERS_PER_DEGREE_LAT), (difference[1] * metersPerLongitudeDegree)])
    return meterCordinates
 
# Converts a point's lat/long into meters using Haversine's formula
# Input:  The set of reference cordinates, the set of point cordinates
# Output: The x & y cordinates of the point
def CalculateHaversine(refLat, refLong, pointLat, pointLong):
    # convert GPS coordinates to radians
    ref_lat_rad     = math.radians(refLat)
    ref_long_rad    = math.radians(refLong)
    point_lat_rad   = math.radians(pointLat)
    point_lon_rad   = math.radians(pointLong)

    # calculate distance and direction from reference point to GPS coordinate
    delta_lon = point_lon_rad - ref_long_rad
    delta_lat = point_lat_rad - ref_lat_rad
    a = math.sin(delta_lat/2)**2 + math.cos(ref_lat_rad) * math.cos(point_lat_rad) * math.sin(delta_lon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = config.EARTH_RADIUS * c
    theta = math.atan2(math.sin(delta_lon) * math.cos(point_lat_rad), math.cos(ref_lat_rad) * math.sin(point_lat_rad) - math.sin(ref_lat_rad) * math.cos(point_lat_rad) * math.cos(delta_lon))

    # convert distance and direction to xy coordinates in meters
    x = d * math.cos(theta)
    y = d * math.sin(theta)
    return x, y

# Converts xy cordinated in meters to latitude/longitude degree coordinates
# Input:  Two np arrays, the first being the center (origin) in Lat/Lang coordinates, the over being the meter coordinates
# output a np array of [latitude, longitude]
def MetersToLatLang(center, point):
    global metersPerLongitudeDegree
    if (metersPerLongitudeDegree == -1):
        __CalculateMetersPerLongitudeDegree(center[0])
    lat = center[0] + (point[1] / config.METERS_PER_DEGREE_LAT)
    long = center[1] + (point[0] / metersPerLongitudeDegree)
    return np.array([lat, long])