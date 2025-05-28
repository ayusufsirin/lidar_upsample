#!/usr/bin/env python

import csv
import os
import time
from collections import deque
from datetime import datetime

import message_filters
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

import threading
import queue


class PointCloudTransformer:
    def __init__(self):
        # Create a unique log file name with timestamp
        self.processing_times = deque(maxlen=100)
        self.input_timestamps = deque(maxlen=100)
        self.processed_timestamps = deque(maxlen=100)
        self.message_count = 0
        self.last_msg_time = None

        # Create ./logs directory relative to script location
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(log_dir, exist_ok=True)

        # Create a unique log file name with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_path = os.path.join(log_dir, f"pointcloud_metrics_{timestamp}.csv")

        # Open the file and create the CSV writer
        self.csv_file = open(self.csv_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write the CSV header
        self.csv_writer.writerow(
            ["ros_time", "pc_timestamp", "latency_sec", "processing_time_sec", "processing_rate_Hz", "input_rate_Hz", "throughput_ratio", "cumulative_points"])

        # Initialize a list to store cumulative transformed points
        self.cumulative_points = []

        # Threading queue
        self.msg_queue = queue.Queue(maxsize=100)
        self.shutdown_flag = threading.Event()

        # Start the processing thread
        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.worker_thread.start()

        # Initialize subscribers
        pc_sub = message_filters.Subscriber('/islam/vlp_pts', PointCloud2)
        odom_sub = message_filters.Subscriber('/islam/vlp_odom', Odometry)

        # ApproximateTime synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([pc_sub, odom_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.synced_callback)

        # Initialize publishers
        self.point_cloud_pub = rospy.Publisher('/transformed_point_cloud', PointCloud2, queue_size=10)
        self.cumulative_cloud_pub = rospy.Publisher('/cumulative_point_cloud', PointCloud2, queue_size=10)
        self.odom_pub = rospy.Publisher('/transformed_odom', Odometry, queue_size=10)

    def calculate_rate(self, timestamps):
        if len(timestamps) < 2:
            return 0.0
        duration = timestamps[-1] - timestamps[0]
        return len(timestamps) / duration if duration > 0 else 0.0

    def synced_callback(self, point_cloud_msg, odom_msg):
        self.input_timestamps.append(rospy.Time.now().to_sec())
        try:
            self.msg_queue.put_nowait((point_cloud_msg, odom_msg))
        except queue.Full:
            rospy.logwarn("Processing queue full — dropping frame.")

    def processing_loop(self):
        while not self.shutdown_flag.is_set() and not rospy.is_shutdown():
            try:
                point_cloud_msg, odom_msg = self.msg_queue.get(timeout=0.1)

                start_time = time.time()

                # Latency
                pc_time = point_cloud_msg.header.stamp.to_sec()
                now = rospy.Time.now().to_sec()
                latency = now - pc_time

                self.processed_timestamps.append(pc_time)
                self.message_count += 1

                translation, rotation, transformed_odom_msg = self.odometry_callback(odom_msg)
                self.point_cloud_callback(point_cloud_msg, translation, rotation)
                self.odom_pub.publish(transformed_odom_msg)

                # Save metrics
                processing_duration = time.time() - start_time
                self.processing_times.append(processing_duration)
                cumulative_count = len(self.cumulative_points)

                processing_rate = self.calculate_rate(self.processed_timestamps)
                input_rate = self.calculate_rate(self.input_timestamps)
                throughput_ratio = processing_rate / input_rate if input_rate > 0 else 0

                # Write to CSV
                self.csv_writer.writerow([
                    now,
                    pc_time,
                    latency,
                    processing_duration,
                    processing_rate,
                    input_rate,
                    throughput_ratio,
                    cumulative_count
                ])
            except queue.Empty:
                continue

    @staticmethod
    def odometry_callback(msg):
        # Extract translation and rotation from the odometry message
        translation = [
            - msg.pose.pose.position.x,
            msg.pose.pose.position.z,
            0,  # msg.pose.pose.position.y,
        ]
        original_rotation = [
            0,  # msg.pose.pose.orientation.x,
            0,  # msg.pose.pose.orientation.z,
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
        rotation = transformations.quaternion_from_matrix(combined_rotation_matrix)

        # Create and publish the transformed odometry message
        transformed_odom_msg = Odometry()
        transformed_odom_msg.header = msg.header
        transformed_odom_msg.child_frame_id = msg.child_frame_id

        # Set the transformed pose in the odometry message
        transformed_odom_msg.pose.pose.position.x = translation[0]
        transformed_odom_msg.pose.pose.position.y = translation[1]
        transformed_odom_msg.pose.pose.position.z = translation[2]

        transformed_odom_msg.pose.pose.orientation.x = rotation[0]
        transformed_odom_msg.pose.pose.orientation.y = rotation[1]
        transformed_odom_msg.pose.pose.orientation.z = rotation[2]
        transformed_odom_msg.pose.pose.orientation.w = rotation[3]

        # Copy the twist part from the original odometry message
        transformed_odom_msg.twist = msg.twist

        return translation, rotation, transformed_odom_msg

    def point_cloud_callback(self, point_cloud_msg, translation, rotation):
        if translation is None or rotation is None:
            rospy.logwarn("Odometry data not yet available, skipping point cloud transformation.")
            return

        # Convert the PointCloud2 message to a list of points
        point_list = list(pc2.read_points(point_cloud_msg, skip_nans=True, field_names=("x", "y", "z")))

        # Transform the point cloud using odometry data
        transformed_points = self.transform_point_cloud(point_list, translation, rotation)

        # Add additional fields from the original point cloud
        new_points = []
        for i, original_point in enumerate(pc2.read_points(point_cloud_msg, skip_nans=True)):
            new_point = list(transformed_points[i]) + list(original_point[3:])
            new_points.append(new_point)

        # Create a new PointCloud2 message with the transformed points
        transformed_msg = pc2.create_cloud(point_cloud_msg.header, point_cloud_msg.fields, new_points)

        # Publish the transformed point cloud
        self.point_cloud_pub.publish(transformed_msg)

        # Add the new points to the cumulative point cloud
        self.cumulative_points.extend(new_points)

        # Create and publish the cumulative PointCloud2 message
        cumulative_msg = pc2.create_cloud(point_cloud_msg.header, point_cloud_msg.fields, self.cumulative_points)
        self.cumulative_cloud_pub.publish(cumulative_msg)

    def transform_point_cloud(self, point_cloud, translation, rotation):
        # Convert input list to a (N, 3) NumPy array
        pc_np = np.array(point_cloud)  # shape: (N, 3)

        if pc_np.shape[0] == 0:
            return []

        # Add homogeneous column (1s)
        ones = np.ones((pc_np.shape[0], 1))
        pc_homogeneous = np.hstack((pc_np, ones))  # shape: (N, 4)

        # Rotation matrix from quaternion
        rotation_matrix = transformations.quaternion_matrix(rotation)  # shape: (4, 4)

        # Apply transformation to all points at once
        transformed_h = pc_homogeneous @ rotation_matrix.T  # shape: (N, 4)

        # Apply translation
        translated = transformed_h[:, :3] + np.array(translation)  # shape: (N, 3)

        return translated


if __name__ == "__main__":
    rospy.init_node('point_cloud_transformer')
    transformer = PointCloudTransformer()


    def shutdown_hook():
        rospy.loginfo("Shutting down, closing CSV file.")
        transformer.shutdown_flag.set()
        transformer.worker_thread.join()
        transformer.csv_file.close()


    rospy.on_shutdown(shutdown_hook)
    rospy.spin()
