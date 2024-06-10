from collections import namedtuple

Header = namedtuple("Header", ['stamp', 'frame_id'])
MapMetaData = namedtuple("MapMetaData", ['map_load_time', 'resolution', 'width', 'height', 'origin'])
OccupancyGrid = namedtuple("OccupancyGrid", ['header', 'info', 'data'])
Odometry = namedtuple("Odometry", ['header', 'frame_id', 'pose', 'twist'])
Point = namedtuple("Point", ['x', 'y', 'z'])
Pose = namedtuple("Pose", ['position', 'orientation'])
PoseWithCovariance = namedtuple("PoseWithCovariance", ['pose', 'covariance'])
Time = namedtuple("Time", ['sec', 'nanosec'])
Twist = namedtuple("Twist", ['linear', 'angular'])
TwistWithCovariance = namedtuple("TwistWithCovariance", ['pose', 'covariance'])
LaserScan = namedtuple("LaserScan", ['header', 'angle_min', 'angle_max', 'angle_increment',
                                        'time_increment', 'scan_time', 'range_min',
                                        'range_max', 'ranges', 'intensities'])

