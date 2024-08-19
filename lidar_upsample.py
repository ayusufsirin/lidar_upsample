#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as transformations
import numpy as np

class PointCloudTransformer:
    def __init__(self):
        # Initialize variables to store odometry data
        self.translation = None
        self.rotation = None

        # Initialize subscribers
        rospy.Subscriber('/islam/vlp_pts', PointCloud2, self.point_cloud_callback)
        rospy.Subscriber('/islam/vlp_odom', Odometry, self.odometry_callback)

        # Initialize publishers
        self.point_cloud_pub = rospy.Publisher('/transformed_point_cloud', PointCloud2, queue_size=10)
        self.odom_pub = rospy.Publisher('/transformed_odom', Odometry, queue_size=10)

    def odometry_callback(self, msg):
        # Extract translation and rotation from the odometry message
        self.translation = [
            - msg.pose.pose.position.x, 
            msg.pose.pose.position.z, 
            0, # msg.pose.pose.position.y,
        ]
        original_rotation = [
            0, # msg.pose.pose.orientation.x,
            0, # msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.w
        ]
                
        # Convert the quaternion to a rotation matrix
        original_rotation_matrix = transformations.quaternion_matrix(original_rotation)

        # Define the additional 90-degree rotations around x, y, and z axes
        rot_x = transformations.rotation_matrix(np.radians(0), [1, 0, 0])
        rot_y = transformations.rotation_matrix(np.radians(0), [0, 1, 0])
        rot_z = transformations.rotation_matrix(np.radians(90), [0, 0, 1])

        # Combine the original rotation with the additional rotations
        combined_rotation_matrix = np.dot(rot_z, np.dot(rot_y, np.dot(rot_x, original_rotation_matrix)))

        # Convert the combined rotation matrix back to a quaternion
        self.rotation = transformations.quaternion_from_matrix(combined_rotation_matrix)
        
        # Create and publish the transformed odometry message
        transformed_odom_msg = Odometry()
        transformed_odom_msg.header = msg.header
        transformed_odom_msg.child_frame_id = msg.child_frame_id

        # Set the transformed pose in the odometry message
        transformed_odom_msg.pose.pose.position.x = self.translation[0]
        transformed_odom_msg.pose.pose.position.y = self.translation[1]
        transformed_odom_msg.pose.pose.position.z = self.translation[2]

        transformed_odom_msg.pose.pose.orientation.x = self.rotation[0]
        transformed_odom_msg.pose.pose.orientation.y = self.rotation[1]
        transformed_odom_msg.pose.pose.orientation.z = self.rotation[2]
        transformed_odom_msg.pose.pose.orientation.w = self.rotation[3]

        # Copy the twist part from the original odometry message
        transformed_odom_msg.twist = msg.twist

        # Publish the transformed odometry message
        self.odom_pub.publish(transformed_odom_msg)

    def point_cloud_callback(self, point_cloud_msg):
        if self.translation is None or self.rotation is None:
            rospy.logwarn("Odometry data not yet available, skipping point cloud transformation.")
            return

        # Convert the PointCloud2 message to a list of points
        point_list = list(pc2.read_points(point_cloud_msg, skip_nans=True, field_names=("x", "y", "z")))

        # Transform the point cloud using odometry data
        transformed_points = self.transform_point_cloud(point_list, self.translation, self.rotation)

        # Add additional fields from the original point cloud
        new_points = []
        for i, original_point in enumerate(pc2.read_points(point_cloud_msg, skip_nans=True)):
            new_point = list(transformed_points[i]) + list(original_point[3:])
            new_points.append(new_point)

        # Create a new PointCloud2 message with the transformed points
        transformed_msg = pc2.create_cloud(point_cloud_msg.header, point_cloud_msg.fields, new_points)

        # Publish the transformed point cloud
        self.point_cloud_pub.publish(transformed_msg)

    def transform_point_cloud(self, point_cloud, translation, rotation):
        # Create a rotation matrix from the quaternion
        rotation_matrix = transformations.quaternion_matrix(rotation)

        # Apply rotation and translation to each point
        transformed_points = []
        for point in point_cloud:
            # Convert point to homogeneous coordinates
            point_h = np.array([point[0], point[1], point[2], 1.0])

            # Apply rotation
            rotated_point_h = np.dot(rotation_matrix, point_h)

            # Apply translation
            translated_point = rotated_point_h[:3] + np.array(translation)

            transformed_points.append(translated_point)

        return transformed_points

if __name__ == "__main__":
    rospy.init_node('point_cloud_transformer')
    transformer = PointCloudTransformer()
    rospy.spin()
