import json
import math


def yaw_from_quaternion_values(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


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
        self.pose_key = node.declare_parameter("redis_pose_key", "warthog:odom").value
        self.timeout_s = float(node.declare_parameter("redis_timeout_s", 0.1).value)
        self.frame_id = node.declare_parameter("redis_frame_id", "odom").value
        self.yaw_units = node.declare_parameter("redis_yaw_units", "rad").value
        self.x_field = node.declare_parameter("redis_x_field", "x").value
        self.y_field = node.declare_parameter("redis_y_field", "y").value
        self.z_field = node.declare_parameter("redis_z_field", "z").value
        self.yaw_field = node.declare_parameter("redis_yaw_field", "yaw").value
        self.time_field = node.declare_parameter("redis_time_field", "time_sec").value
        self.frame_field = node.declare_parameter("redis_frame_field", "frame_id").value

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
            decode_responses=True,
        )

    def get_pose(self):
        data = self.read_key()
        if not data:
            return None

        try:
            x = float(self.lookup(data, self.x_field, ["pose.x", "position.x"]))
            y = float(self.lookup(data, self.y_field, ["pose.y", "position.y"]))
            z = float(self.lookup_optional(data, self.z_field, ["pose.z", "position.z"], 0.0))
            yaw = self.parse_yaw(data)
        except (TypeError, ValueError, KeyError):
            return None

        if self.yaw_units.lower() in ("deg", "degree", "degrees"):
            yaw = math.radians(yaw)

        time_value = self.lookup_optional(data, self.time_field, ["stamp", "timestamp"], None)
        time_sec = self.node.get_clock().now().nanoseconds * 1e-9
        if time_value not in (None, ""):
            try:
                time_sec = float(time_value)
            except ValueError:
                pass

        frame_id = self.lookup_optional(data, self.frame_field, ["header.frame_id"], self.frame_id)
        return RedisPose(time_sec, frame_id or self.frame_id, x, y, z, yaw)

    def read_key(self):
        key_type = self.client.type(self.pose_key)
        if key_type == "hash":
            return self.client.hgetall(self.pose_key)

        value = self.client.get(self.pose_key)
        if value is None:
            return None
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError:
            return None
        return parsed if isinstance(parsed, dict) else None

    def parse_yaw(self, data):
        yaw = self.lookup_optional(data, self.yaw_field, ["pose.yaw", "orientation.yaw"], None)
        if yaw not in (None, ""):
            return float(yaw)

        qx = self.lookup_optional(data, "qx", ["orientation.x", "pose.orientation.x"], None)
        qy = self.lookup_optional(data, "qy", ["orientation.y", "pose.orientation.y"], None)
        qz = self.lookup_optional(data, "qz", ["orientation.z", "pose.orientation.z"], None)
        qw = self.lookup_optional(data, "qw", ["orientation.w", "pose.orientation.w"], None)
        if None in (qx, qy, qz, qw):
            raise KeyError("yaw")
        return yaw_from_quaternion_values(float(qx), float(qy), float(qz), float(qw))

    def lookup(self, data, field_name, fallback_paths=None):
        value = self.lookup_optional(data, field_name, fallback_paths, None)
        if value is None:
            raise KeyError(field_name)
        return value

    def lookup_optional(self, data, field_name, fallback_paths=None, default=None):
        paths = [field_name]
        if fallback_paths:
            paths.extend(fallback_paths)

        for path in paths:
            value = self.lookup_path(data, path)
            if value is not None:
                return value
        return default

    @staticmethod
    def lookup_path(data, path):
        if not path:
            return None

        current = data
        for part in str(path).split("."):
            if not isinstance(current, dict) or part not in current:
                return None
            current = current[part]
        return current
