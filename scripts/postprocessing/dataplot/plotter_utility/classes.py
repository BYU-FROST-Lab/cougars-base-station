class IMU:
    # 1680132778.5222569,IMU,"[-0.032494, 0.003320, -0.037022, 0.998781][-0.000414, -0.000398, 0.007080][-0.001737, 0.833299, -9.800618]"
    def __init__(self, row):
        self.timestamp = float(row[0])
        self.quaternion = row[2].split('][')[0]
        self.angular_velocity = row[2].split('][')[1]
        self.acceleration = row[2].split('][')[2]

class GPS:
    def __init__(self, row):
        self.timestamp = float(row[0])
        self.gps_lat = float(row[2].split(', ')[0])
        self.gps_long = float(row[2].split(', ')[1])
        self.elevation = float(row[2].split(', ')[2])
        

class Heading:
    def __init__(self, row):
        self.timestamp = float(row[0])
    
        # 1680132778.6011596,"Lat: 40.14989084 Long: -111.55759938 Heading: 88.07599639892578 Heading Rate: 0.2275215404464499             Speed Over Ground: 3.30807646138752             Course Over Ground: 88.21534729003906             GPS Fix"" 1             GPS fix week num: 2255             GPS fix ms: 343996506             Num Sattelites: 3             Position Uncertainty: 0.0"
        # remove whitespace
        line = row[1].split(' ')
        # remove all instances of (' ')
        line = [x for x in line if x != '']
        self.lat = float(line[1])
        self.long = float(line[3])
        self.heading = float(line[5])
        self.heading_rate = float(line[8])
        self.speed_over_ground = float(line[12])
        self.course_over_ground = float(line[16])
        self.gps_fix = float(line[19])
        self.gps_fix_week_num = float(line[24])
        self.gps_fix_ms = float(line[28])
        self.num_sattelites = float(line[31])
        self.position_uncertainty = float(line[34])

class BEAM_RANGE:
    def __init__(self, row):
        self.timestamp = float(row[0])
        beam_range = row[2][12:-2].split(',')
        beam_range = [float(i) for i in beam_range]
        self.beam_range = beam_range

class BEAM_INTENSITY:
    def __init__(self, row):
        self.timestamp = float(row[0])
        beam_int = row[2][12:-2].split(',')
        beam_int = [float(i) for i in beam_int]
        self.beam_intensity = beam_int
