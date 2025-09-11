#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2

class TopicRemapper:
    def __init__(self):
        # Publishers for remapped topics
        self.imu_pub_correct = rospy.Publisher('/imu_correct', Imu, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu_raw', Imu, queue_size=10)

        # self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.points_pub = rospy.Publisher('/points_raw', PointCloud2, queue_size=10)

        # Subscribers to original topics
        rospy.Subscriber('/vectornav/IMU', Imu, self.imu_callback)
        # rospy.Subscriber('/vectornav/GPS', NavSatFix, self.gps_callback)
        rospy.Subscriber('/ouster/points', PointCloud2, self.points_callback)

    def imu_callback(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'imu_link'
        self.imu_pub.publish(msg)

        msg.header.frame_id = 'base_link'
        # msg.orientation.x = -msg.orientation.y
        # msg.orientation.y = msg.orientation.x
        # msg.linear_acceleration.x = -msg.linear_acceleration.x
        # msg.linear_acceleration.z = -msg.linear_acceleration.z
        # msg.angular_velocity.x = -msg.angular_velocity.x
        # msg.angular_velocity.z = -msg.angular_velocity.z
        self.imu_pub_correct.publish(msg)
        
    def gps_callback(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'navsat'
        self.gps_pub.publish(msg)

    def points_callback(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'os_sensor'
        self.points_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('topic_remapper')
    TopicRemapper()
    rospy.spin()



# import rospy
# import sensor_msgs.point_cloud2 as pc2
# import pcl
# from sensor_msgs.msg import PointCloud2
# import std_msgs.msg
# import numpy as np

# def filter_nan_points(cloud_data):
#     # Convert PointCloud2 message to a list of points
#     pc_data = list(pc2.read_points(cloud_data, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=False))
    
#     # Convert to numpy array for easier processing
#     pc_array = np.array(pc_data)
    
#     # Filter out rows where x, y, or z are NaN
#     pc_array = pc_array[~np.isnan(pc_array).any(axis=1)]  # Remove any rows with NaN values
    
#     # Convert back to a PointCloud2 message with the filtered data
#     new_cloud_data = pc2.create_cloud_xyz32(cloud_data.header, pc_array[:, 0:3])  # Only take x, y, z
#     return new_cloud_data

# def point_cloud_callback(msg):
#     rospy.loginfo("Received PointCloud2 data")

#     # Step 1: Remove NaN values from the original PointCloud2 message
#     filtered_pc = filter_nan_points(msg)
    
#     # Step 2: Convert to the LIO-SAM compatible format (ensure it's compatible with x, y, z, intensity, ring)
#     # We can add intensity and ring here if needed
#     filtered_pc.fields = [
#         std_msgs.msg.Field(name="x", offset=0, datatype=std_msgs.msg.Field.FLOAT32, count=1),
#         std_msgs.msg.Field(name="y", offset=4, datatype=std_msgs.msg.Field.FLOAT32, count=1),
#         std_msgs.msg.Field(name="z", offset=8, datatype=std_msgs.msg.Field.FLOAT32, count=1),
#         std_msgs.msg.Field(name="intensity", offset=12, datatype=std_msgs.msg.Field.FLOAT32, count=1),
#         std_msgs.msg.Field(name="ring", offset=16, datatype=std_msgs.msg.Field.UINT32, count=1)
#     ]
    
#     # Step 3: You can now publish the filtered and converted point cloud to a new topic
#     pub.publish(filtered_pc)

# # Initialize the ROS node
# rospy.init_node('point_cloud_filter_node')

# # Create a subscriber to your original PointCloud2 topic
# rospy.Subscriber("/os_cloud_node/points", PointCloud2, point_cloud_callback)

# # Create a publisher for the filtered PointCloud2 message
# pub = rospy.Publisher("/filtered_points", PointCloud2, queue_size=10)

# # Spin to keep the node running
# rospy.spin()
