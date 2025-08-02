#!/usr/bin/env python

import csv
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime

import cupy as cp
import message_filters
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

PC_HISTORY_SIZE = 10
PC_TOPIC = '/velodyne_points'
ODOM_TOPIC = '/jackal_velocity_controller/odom'

TRANSFORMED_POINT_CLOUD = '/transformed_point_cloud'
CUMULATIVE_POINT_CLOUD = '/cumulative_point_cloud'
CUMULATIVE_ORIGIN_POINT_CLOUD = '/cumulative_origin_point_cloud'

rospy.set_param('/use_sim_time', True)

# %% Faster PC creation from NP
def create_cloud_from_np(header, fields, np_array):
    """
    Fast version of create_cloud, using NumPy vectorized byte representation.
    Assumes np_array is (N, 3) float32 for (x, y, z).
    """

    # Flatten the array to 1D byte representation
    data = np_array.astype(np.float32).tobytes()

    cloud_msg = PointCloud2()
    cloud_msg.header = header
    cloud_msg.height = 1
    cloud_msg.width = np_array.shape[0]
    cloud_msg.fields = fields
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 12  # 3 floats * 4 bytes
    cloud_msg.row_step = cloud_msg.point_step * np_array.shape[0]
    cloud_msg.is_dense = True
    cloud_msg.data = data

    return cloud_msg


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
        self.csv_writer.writerow([
            "ros_time",
            "pc_timestamp",
            "latency_ms",
            "processing_time_ms",
            'odom_callback_duration_ms',
            'pc_callback_duration_ms',
            'pc_to_points_duration_ms',
            'transform_points_duration_ms',
            'pc_create_duration_ms',
            'cumulative_points_duration_ms',
            'cumulative_points_create_cloud_duration_ms',
            'translate_points_duration_ms',
            'cumulative_origin_points_duration_ms',
            'cumulative_origin_points_create_cloud_duration_ms',
            "processing_rate_Hz",
            "input_rate_Hz",
            "throughput_ratio",
            "cumulative_points",
            "input_queue_size"
        ])

        # Initialize deques to store cumulative transformed points with a maximum length
        # This prevents unbounded memory growth by keeping only the most recent PC_HISTORY_SIZE point clouds.
        self.cumulative_points = deque(maxlen=PC_HISTORY_SIZE)
        self.cumulative_origin_points = deque(maxlen=PC_HISTORY_SIZE)

        # Threading queue
        self.msg_queue = queue.Queue(maxsize=100)
        self.shutdown_flag = threading.Event()

        # Start the processing thread
        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.worker_thread.start()

        # Initialize subscribers
        pc_sub = message_filters.Subscriber(PC_TOPIC, PointCloud2)
        odom_sub = message_filters.Subscriber(ODOM_TOPIC, Odometry)

        # ApproximateTime synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([pc_sub, odom_sub], queue_size=10, slop=0.1, reset=True)
        ts.registerCallback(self.synced_callback)

        # Initialize publishers
        self.point_cloud_pub = rospy.Publisher(TRANSFORMED_POINT_CLOUD, PointCloud2, queue_size=10)
        self.cumulative_cloud_pub = rospy.Publisher(CUMULATIVE_POINT_CLOUD, PointCloud2, queue_size=10)
        self.cumulative_origin_cloud_pub = rospy.Publisher(CUMULATIVE_ORIGIN_POINT_CLOUD, PointCloud2,
                                                           queue_size=10)  # NEW PUBLISHER
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
                latency_ms = (now - pc_time) * 1000.0

                self.processed_timestamps.append(pc_time)
                self.message_count += 1

                odom_callback_start_time = time.time()
                translation, rotation, transformed_odom_msg = self.odometry_callback(odom_msg)
                odom_callback_duration_ms = (time.time() - odom_callback_start_time) * 1000.0

                pc_callback_start_time = time.time()
                pc_callback_stats = self.point_cloud_callback(point_cloud_msg, translation, rotation)
                pc_callback_duration_ms = (time.time() - pc_callback_start_time) * 1000.0

                self.odom_pub.publish(transformed_odom_msg)

                # Save metrics
                processing_duration_ms = (time.time() - start_time) * 1000.0
                self.processing_times.append(processing_duration_ms)
                cumulative_count = len(self.cumulative_points)

                processing_rate = self.calculate_rate(self.processed_timestamps)
                input_rate = self.calculate_rate(self.input_timestamps)
                throughput_ratio = processing_rate / input_rate if input_rate > 0 else 0

                # Write to CSV
                self.csv_writer.writerow([
                    now,
                    pc_time,
                    latency_ms,
                    processing_duration_ms,
                    odom_callback_duration_ms,
                    pc_callback_duration_ms,
                    *pc_callback_stats.values(),
                    processing_rate,
                    input_rate,
                    throughput_ratio,
                    cumulative_count,
                    self.msg_queue.qsize()  # approx.
                ])
            except queue.Empty:
                continue

    @staticmethod
    def odometry_callback(msg):
        # Extract translation
        translation = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]

        # Use identity matrix for no rotation
        combined_rotation_matrix = np.eye(4)

        # Convert identity rotation matrix back to a quaternion
        rotation = transformations.quaternion_from_matrix(combined_rotation_matrix)

        # Create and publish the transformed odometry message
        transformed_odom_msg = Odometry()
        transformed_odom_msg.header = msg.header
        transformed_odom_msg.child_frame_id = msg.child_frame_id

        transformed_odom_msg.pose.pose.position.x = translation[0]
        transformed_odom_msg.pose.pose.position.y = translation[1]
        transformed_odom_msg.pose.pose.position.z = translation[2]

        transformed_odom_msg.pose.pose.orientation.x = rotation[0]
        transformed_odom_msg.pose.pose.orientation.y = rotation[1]
        transformed_odom_msg.pose.pose.orientation.z = rotation[2]
        transformed_odom_msg.pose.pose.orientation.w = rotation[3]

        transformed_odom_msg.twist = msg.twist

        return translation, rotation, transformed_odom_msg

    def point_cloud_callback(self, point_cloud_msg, translation, rotation):
        rospy.loginfo("point_cloud_callback")

        if translation is None or rotation is None:
            rospy.logwarn("Odometry data not yet available, skipping point cloud transformation.")
            return

        # %% Step 1: Convert PointCloud2 to list of full points (all fields)
        pc_to_points_start_time = time.time()
        xyz_cp = cp.asarray(list(pc2.read_points(point_cloud_msg, skip_nans=True, field_names=("x", "y", "z"))),
                       dtype=cp.float32)
        pc_to_points_duration_ms = (time.time() - pc_to_points_start_time) * 1000.0

        # %% Step 2: Transform Points with CuPy
        transform_points_start_time = time.time()
        ones = cp.ones((xyz_cp.shape[0], 1), dtype=cp.float32)
        homogeneous = cp.concatenate((xyz_cp, ones), axis=1)

        rot_mat_cp = cp.asarray(transformations.quaternion_matrix(rotation), dtype=cp.float32)
        transformed_cp = homogeneous @ rot_mat_cp.T
        transformed_xyz = transformed_cp[:, :3] + cp.asarray(translation, dtype=cp.float32)
        transformed_points = transformed_xyz.get()
        transform_points_duration_ms = (time.time() - transform_points_start_time) * 1000.0

        # %% Step 4: Create and publish transformed point cloud
        pc_create_start_time = time.time()
        transformed_msg = create_cloud_from_np(
            point_cloud_msg.header,
            point_cloud_msg.fields,
            transformed_points
        )
        pc_create_duration_ms = (time.time() - pc_create_start_time) * 1000.0
        self.point_cloud_pub.publish(transformed_msg)

        # %% Step 5: Update cumulative transformed cloud
        cumulative_points_start_time = time.time()
        self.cumulative_points.append(transformed_points)
        points_cumulative_transformed = np.vstack(self.cumulative_points)
        cumulative_points_create_cloud_start_time = time.time()
        cumulative_msg = create_cloud_from_np(
            point_cloud_msg.header,
            point_cloud_msg.fields,
            points_cumulative_transformed
        )
        cumulative_points_create_cloud_duration_ms = (time.time() - cumulative_points_create_cloud_start_time) * 1000.0
        self.cumulative_cloud_pub.publish(cumulative_msg)
        cumulative_points_duration_ms = (time.time() - cumulative_points_start_time) * 1000.0

        # %% Step 6: Translate points back to origin
        translate_points_start_time = time.time()
        translated_to_origin = transformed_points - np.array(translation, dtype=np.float32)
        translate_points_duration_ms = (time.time() - translate_points_start_time) * 1000.0

        # %% Step 7: Cumulative origin-aligned cloud
        cumulative_origin_points_start_time = time.time()
        self.cumulative_origin_points.append(translated_to_origin)
        points_cumulative_origin = np.vstack(self.cumulative_origin_points)
        cum_origin_create_cloud_start_time = time.time()
        cumulative_origin_msg = create_cloud_from_np(
            point_cloud_msg.header,
            point_cloud_msg.fields,
            points_cumulative_origin
        )
        cum_origin_points_create_cloud_duration_ms = (time.time() - cum_origin_create_cloud_start_time) * 1000.0

        self.cumulative_origin_cloud_pub.publish(cumulative_origin_msg)
        cumulative_origin_points_duration_ms = (time.time() - cumulative_origin_points_start_time) * 1000.0

        return {
            'pc_to_points_duration_ms': pc_to_points_duration_ms,
            'transform_points_duration_ms': transform_points_duration_ms,
            'pc_create_duration_ms': pc_create_duration_ms,
            'cumulative_points_duration_ms': cumulative_points_duration_ms,
            'cumulative_points_create_cloud_duration_ms': cumulative_points_create_cloud_duration_ms,
            'translate_points_duration_ms': translate_points_duration_ms,
            'cumulative_origin_points_duration_ms': cumulative_origin_points_duration_ms,
            'cumulative_origin_points_create_cloud_duration_ms': cum_origin_points_create_cloud_duration_ms,
        }

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
