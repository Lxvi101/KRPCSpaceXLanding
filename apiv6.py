from enum import Enum
import datetime
import os
import time
from math import sin, cos, pi
import math

class FlightPhase(Enum):
    IDLE = -1
    START = 0
    FLIGHT = 1
    BOOSTBACK = 2
    RETURN = 3
    BURN = 4
    LANDING = 5
    LANDED = 6

class Logger:
    def __init__(self):
        self.last_log_time = 0  # Initialize with 0

        if not os.path.exists("flightlogs"):
            os.makedirs("flightlogs")
        
        self.log_filename = datetime.datetime.now().strftime("flightlog_%Y%m%d_%H%M%S.txt")

    def log_strict(self, message):
        """Logs a message to the specified file within the 'flightlogs' directory with a timestamp."""

        filepath = os.path.join("flightlogs", self.log_filename)
        
        with open(filepath, 'a') as file:
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            file.write(f"{timestamp} - {message}\n")
            
    def log(self, message):
        """Logs the message, but ensures at least a 1 second delay between log calls."""
        current_time = time.time()
        if current_time - self.last_log_time >= 1:  # Ensure at least 1 second has passed
            self.log_strict(f"METRIC LOG > {message}")
            self.last_log_time = current_time

    def log_timout(self, message, timeout):
        """Logs the message, but ensures at least an x second delay between log calls."""
        current_time = time.time()
        if current_time - self.last_log_time >= timeout:  # Ensure at least 1 second has passed
            self.log_strict(f"METRIC LOG > {message}")
            self.last_log_time = current_time


class TargetManager():
    def __init__(self, connection, lat = -1.5179802160180738, lon = -71.89893447788006): # Island runway coordinates
        self.target_lat_lon = (lat,lon)
        self.connection = connection
        self.vessel = connection.space_center.active_vessel
        self.orbit = self.vessel.orbit
        self.celestial_body = self.vessel.orbit.body

    def target(self, lat, lon, alt = 0, celestial_body = None):
        self.target_lat_lon = (lat,lon)
        if celestial_body != None:
            self.celestial_body = celestial_body

    def get_target_position(self, ref_frame = None):
        if not ref_frame:
            ref_frame = self.celestial_body.reference_frame
        return self.celestial_body.surface_position(self.target_lat_lon[0], self.target_lat_lon[1], ref_frame)
    
    def define_start_pad_ref_frame(self):
        _ref = self.connection.space_center.ReferenceFrame.create_relative(self.orbit.body.reference_frame, self.vessel.position(self.orbit.body.reference_frame))
        ref_frame = self.connection.space_center.ReferenceFrame.create_hybrid(
            position=_ref,
            rotation=self.vessel.surface_reference_frame)
        self.start_pad_ref_frame = ref_frame
        return ref_frame
    
    def define_landing_pad_ref_frame(self):
        # Crazy quarternion action incoming
        create_relative = self.connection.space_center.ReferenceFrame.create_relative
        TargetPosition_alt = 0

        q_long = (
            0,
            sin(-self.target_lat_lon[1] * 0.5 * pi / 180),
            0,
            cos(-self.target_lat_lon[1] * 0.5 * pi / 180)
        )
        q_lat = (
            0,
            0,
            sin(self.target_lat_lon[0] * 0.5 * pi / 180),
            cos(self.target_lat_lon[0] * 0.5 * pi / 180)
        )
        target_ref_frame = \
            create_relative(
                create_relative(
                    create_relative(
                        self.celestial_body.reference_frame,
                        self.get_target_position(),
                        q_long),
                    (0, 0, 0),
                    q_lat),
                (TargetPosition_alt, 0, 0))
        
        self.landing_pad_ref_frame = target_ref_frame
        return target_ref_frame

    def get_pointing_ref_frame(self):
        if not self.landing_pad_ref_frame:
            self.define_landing_pad_ref_frame()
        if not self.start_pad_ref_frame:
            self.start_pad_ref_frame()

        return create_reference_frame_pointing_to_another(self.connection, self.vessel, self.start_pad_ref_frame, self.landing_pad_ref_frame)
    
    def calculate_required_heading(self):
        north = self.celestial_body.direction(self.landing_pad_ref_frame)
        vessel_direction = self.vessel.position(self.landing_pad_ref_frame)
        horizon_direction = (0, vessel_direction[1], vessel_direction[2])
        current_heading = angle_between_vectors(north, horizon_direction)
        if horizon_direction[2] < 0:
            current_heading = 360 - current_heading
        return current_heading
    
    def calculate_error_correction_heading(self, error):
        """ Error in landing ref frame """
        north = self.celestial_body.direction(self.landing_pad_ref_frame)
        vessel_direction = error
        horizon_direction = (0, vessel_direction[1], vessel_direction[2])
        current_heading = angle_between_vectors(north, horizon_direction)
        if horizon_direction[2] < 0:
            current_heading = 360 - current_heading
        return current_heading


class ImpactManager():
    def __init__(self, connection):
        self.connection = connection
        self.vessel = connection.space_center.active_vessel
        self.celestial_body = self.vessel.orbit.body

    def get_impact_lat_lon(self):
        """ Interfaces with my custom coded KSP mod for more accuracy with exellent error handling :P """
        impact = 0
        try:
            impact = self.connection.trajectory.get_impact_lat_lon()
            self.impact_lat_lon = impact
        except Exception as e:
            impact = self.impact_lat_lon
        return impact
    
    def calculate_impact(self, ref_frame):
        impactLL = self.get_impact_lat_lon()
        return self.celestial_body.surface_position(impactLL[0],impactLL[1], ref_frame)
        
class SuicideBurnManager():
    def __init__(self, connection):
        self.connection = connection
        self.vessel = connection.space_center.active_vessel
        self.celestial_body = self.vessel.orbit.body

    def update_calculations(self, h, v):
        g = (self.vessel.orbit.body.gravitational_parameter)/pow(self.vessel.orbit.body.equatorial_radius + h, 2)
        a_max = (self.vessel.max_thrust / self.vessel.mass) - g

        a_targeted = (v**2)/(2*h)
        self.throttle_targeted = a_targeted / a_max

        self.height_targeted = (self.vessel.mass / self.vessel.max_thrust) * (0.5*v**2 + g * h)


#region REFERENCE FRAME MANIPULATION #

def create_reference_frame_pointing_to_another(conn, vessel, origin_frame, target_frame):
    """
    Create a custom reference frame where the x-axis (forward direction) 
    points from the origin of the 'origin_frame' to the origin of the 'target_frame'.
    
    Parameters:
    - vessel: The vessel object (used to access certain properties).
    - origin_frame: The reference frame from which the direction is computed.
    - target_frame: The reference frame to which the direction points.
    
    Returns:
    - The custom reference frame.
    """
    # Compute the direction vector from the origin of the origin_frame to the origin of the target_frame
    #direction_vector = tuple(a - b for a, b in zip(vessel.position(target_frame), vessel.position(origin_frame)))
    direction_vector = conn.space_center.transform_position((0,0,0), target_frame, origin_frame)

    # Normalize this direction vector
    magnitude = (sum([v**2 for v in direction_vector]))**0.5
    forward = tuple(v/magnitude for v in direction_vector)
    
    # Choose an arbitrary "up" vector (can be the vessel's "up" or the body's "north", for example)
    arbitrary_up = (1, 0, 0)  # This is an example; adjust as needed
    
    # Compute the "right" direction as the cross product of "forward" and "arbitrary_up"
    right = (
        forward[1]*arbitrary_up[2] - forward[2]*arbitrary_up[1],
        forward[2]*arbitrary_up[0] - forward[0]*arbitrary_up[2],
        forward[0]*arbitrary_up[1] - forward[1]*arbitrary_up[0]
    )
    
    # Normalize "right"
    magnitude_right = (sum([r**2 for r in right]))**0.5
    right_normalized = tuple(r/magnitude_right for r in right)
    
    # Compute the actual "up" direction as the cross product of "right" and "forward"
    up = (
        right_normalized[1]*forward[2] - right_normalized[2]*forward[1],
        right_normalized[2]*forward[0] - right_normalized[0]*forward[2],
        right_normalized[0]*forward[1] - right_normalized[1]*forward[0]
    )
    
    # Convert the basis vectors to a quaternion
    rotation_quaternion = vectors_to_quaternion(forward, right_normalized, up)
    
    # Create the custom reference frame
    custom_frame = vessel.reference_frame.create_relative(
        reference_frame=origin_frame,
        position=(0, 0, 0),
        rotation=rotation_quaternion
    )
    
    return custom_frame

def vectors_to_quaternion(forward, right, up):
    """Convert basis vectors to a quaternion."""
    m00, m01, m02 = forward
    m10, m11, m12 = right
    m20, m21, m22 = up
    
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 0.5 / (trace**0.5)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    else:
        if m00 > m11 and m00 > m22:
            s = 2.0 * (m00 - m11 - m22)**0.5
            w = (m21 - m12) / s
            x = 0.25 * s
            y = (m01 + m10) / s
            z = (m02 + m20) / s
        elif m11 > m22:
            s = 2.0 * (m11 - m00 - m22)**0.5
            w = (m02 - m20) / s
            x = (m01 + m10) / s
            y = 0.25 * s
            z = (m12 + m21) / s
        else:
            s = 2.0 * (m22 - m00 - m11)**0.5
            w = (m10 - m01) / s
            x = (m02 + m20) / s
            y = (m12 + m21) / s
            z = 0.25 * s
    return (x, y, z, w)

#endregion REFERENCE FRAME MANIPULATION #

#region VECTOR MATH #

def haversine_distance(lat1, lon1, lat2, lon2, R):
    """Compute the great-circle distance between two points given their latitudes and longitudes."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c

def normalize(vector):
    """Normalize a 3D vector."""
    x, y, z = vector
    magnitude = (x**2 + y**2 + z**2)**0.5

    # Check to avoid division by zero
    if magnitude == 0:
        return (0, 0, 0)
    return (x/magnitude, y/magnitude, z/magnitude)

def cross_product(u, v):
    return (u[1]*v[2] - u[2]*v[1],
            u[2]*v[0] - u[0]*v[2],
            u[0]*v[1] - u[1]*v[0])

def vec_add(u, v):
    return tuple(a + b for a, b in zip(u, v))

def vec_sub(u, v):
    return tuple(a - b for a, b in zip(u, v))

def scalar_div(u, s):
    return tuple(a / s for a in u)

def scalar_mult(u, s):
    return tuple(a * s for a in u)

def dot_product(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]

def magnitude(v):
    return math.sqrt(dot_product(v, v))

def angle_between_vectors(u, v):
    """ Compute the angle between vector u and v """
    dp = dot_product(u, v)
    if dp == 0:
        return 0
    um = magnitude(u)
    vm = magnitude(v)
    return math.acos(dp / (um*vm)) * (180. / math.pi)


#endregion VECTOR MATH #