import math
import xml.etree.ElementTree as ET


class RedisPose:
    def __init__(self, time_sec, frame_id, x, y, z, yaw):
        self.time_sec = time_sec
        self.frame_id = frame_id
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw


class RedisPoseReader:
    def __init__(self, node):
        self.node = node
        self.host = node.declare_parameter("redis_host", "localhost").value
        self.port = int(node.declare_parameter("redis_port", 6379).value)
        self.db = int(node.declare_parameter("redis_db", 0).value)
        self.password = node.declare_parameter("redis_password", "").value
        self.stream_name = node.declare_parameter("redis_stream_name", "warthog:odom").value
        self.target_node = str(node.declare_parameter("redis_target_node", "warthog").value)
        self.xml_field = node.declare_parameter("redis_xml_field", "xml").value
        self.timeout_s = float(node.declare_parameter("redis_timeout_s", 0.1).value)
        self.frame_id = node.declare_parameter("redis_frame_id", "odom").value
        self.yaw_from_motion_min_distance_m = float(
            node.declare_parameter("redis_yaw_from_motion_min_distance_m", 0.05).value
        )

        self.last_xy = None
        self.last_yaw = 0.0

        try:
            import redis
        except ImportError as exc:
            raise RuntimeError(
                "Python Redis client is not installed. Install the python3-redis package."
            ) from exc

        password = self.password if self.password else None
        self.client = redis.Redis(
            host=self.host,
            port=self.port,
            db=self.db,
            password=password,
            socket_timeout=self.timeout_s,
            socket_connect_timeout=self.timeout_s,
        )

    def get_pose(self):
        latest = self.client.xrevrange(self.stream_name, "+", count=1)
        if not latest:
            return None

        entry_id, fields = latest[0]
        bytes = self.lookup_field(fields, self.xml_field)
        if bytes is None:
            return None

        pose_values = self.get_pose_from_redis(bytes)
        if pose_values is None or len(pose_values) < 3:
            return None

        x = float(pose_values[0])
        y = float(pose_values[1])
        z = float(pose_values[2])
        yaw = float(pose_values[3]) if len(pose_values) >= 4 else self.yaw_from_motion(x, y)
        time_sec = self.time_from_entry_id(entry_id)
        return RedisPose(time_sec, self.frame_id, x, y, z, yaw)



    @staticmethod
    def parse_pose(pose_text):
        if pose_text is None:
            return None

        try:
            return tuple(float(value.strip()) for value in pose_text.strip("()").split(","))
        except ValueError:
            return None

    @staticmethod
    def lookup_field(fields, field_name):
        if field_name in fields:
            return fields[field_name]

        field_bytes = field_name.encode()
        if field_bytes in fields:
            return fields[field_bytes]

        return None

    def yaw_from_motion(self, x, y):
        if self.last_xy is not None:
            last_x, last_y = self.last_xy
            distance = math.hypot(x - last_x, y - last_y)
            if distance >= self.yaw_from_motion_min_distance_m:
                self.last_yaw = math.atan2(y - last_y, x - last_x)

        self.last_xy = (x, y)
        return self.last_yaw

    def time_from_entry_id(self, entry_id):
        if isinstance(entry_id, bytes):
            entry_id = entry_id.decode()

        try:
            milliseconds = float(str(entry_id).split("-", 1)[0])
            return milliseconds * 1e-3
        except (TypeError, ValueError):
            return self.node.get_clock().now().nanoseconds * 1e-9
