import math


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_vector_by_quaternion(q, vector):
    vx, vy, vz = vector
    qx = q.x
    qy = q.y
    qz = q.z
    qw = q.w

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def ecef_to_geodetic_lat_lon(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))

    for _ in range(6):
        sin_lat = math.sin(lat)
        radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        lat = math.atan2(z + WGS84_E2 * radius * sin_lat, p)

    return lat, lon


class VectornavEcefLocalizer:
    def __init__(self):
        self.origin = None
        self.origin_lat = None
        self.origin_lon = None
        self.last_xy = None
        self.last_yaw = None

    def local_pose(self, pose_msg):
        position = pose_msg.pose.pose.position
        orientation = pose_msg.pose.pose.orientation

        if self.origin is None:
            self.origin = (position.x, position.y, position.z)
            self.origin_lat, self.origin_lon = ecef_to_geodetic_lat_lon(position.x, position.y, position.z)

        x, y, z = self.ecef_to_enu(position.x, position.y, position.z)
        yaw = self.yaw_from_ecef_orientation(orientation)

        if self.last_xy is not None:
            last_x, last_y = self.last_xy
            distance = math.hypot(x - last_x, y - last_y)
            if yaw is None and distance > 0.2:
                yaw = math.atan2(y - last_y, x - last_x)
        elif self.last_yaw is None:
            yaw = None

        self.last_xy = (x, y)
        self.last_yaw = yaw
        return x, y, z, yaw

    def ecef_to_enu(self, x, y, z):
        origin_x, origin_y, origin_z = self.origin
        dx = x - origin_x
        dy = y - origin_y
        dz = z - origin_z

        sin_lat = math.sin(self.origin_lat)
        cos_lat = math.cos(self.origin_lat)
        sin_lon = math.sin(self.origin_lon)
        cos_lon = math.cos(self.origin_lon)

        east = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return east, north, up

    def ecef_vector_to_enu(self, dx, dy, dz):
        sin_lat = math.sin(self.origin_lat)
        cos_lat = math.cos(self.origin_lat)
        sin_lon = math.sin(self.origin_lon)
        cos_lon = math.cos(self.origin_lon)

        east = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return east, north, up

    def yaw_from_ecef_orientation(self, orientation):
        if self.origin is None:
            return None

        forward_ecef = rotate_vector_by_quaternion(orientation, (1.0, 0.0, 0.0))
        forward_enu = self.ecef_vector_to_enu(*forward_ecef)
        east, north, _ = forward_enu
        if math.hypot(east, north) < 1e-6:
            return None
        return math.atan2(north, east)


class OdomVectornavHeadingLocalizer:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.last_odom_xy = None

    def local_pose(self, odom_msg, heading_yaw):
        pose = odom_msg.pose.pose
        odom_x = pose.position.x
        odom_y = pose.position.y
        self.z = pose.position.z

        odom_yaw = quaternion_to_yaw(pose.orientation)
        yaw = heading_yaw if heading_yaw is not None else odom_yaw

        if self.last_odom_xy is None:
            self.last_odom_xy = (odom_x, odom_y)
            return self.x, self.y, self.z, yaw

        last_odom_x, last_odom_y = self.last_odom_xy
        delta_distance = math.hypot(odom_x - last_odom_x, odom_y - last_odom_y)
        self.last_odom_xy = (odom_x, odom_y)

        self.x += delta_distance * math.cos(yaw)
        self.y += delta_distance * math.sin(yaw)
        return self.x, self.y, self.z, yaw
