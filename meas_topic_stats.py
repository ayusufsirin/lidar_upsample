# %% Performance
import csv
import importlib
import logging
import os
import time
from inspect import getmembers

import message_filters
import numpy as np
import rospy

# %%
rospy.init_node('sf', anonymous=True)
rospy.set_param('/rosgraph/log_level', logging.DEBUG)

rospy.loginfo("Node initialized")


# %% Utils

def has_header_field(msg_class):
    return any(name == 'header' for name, _ in getmembers(msg_class))


# %% Topics

def get_all_topic_types():
    """
    Get all topics and their associated message types.
    Returns:
        List of (topic_name, message_type) tuples.
    """
    return rospy.get_published_topics()


def import_msg_class(msg_type):
    """
    Import a message class dynamically from a string like 'sensor_msgs/Image'.
    Returns:
        Python class of the message or None if it fails.
    """
    try:
        pkg_name, msg_name = msg_type.split('/')
        module = importlib.import_module(f"{pkg_name}.msg")
        return getattr(module, msg_name)
    except Exception as e:
        rospy.logwarn(f"Failed to import {msg_type}: {e}")
        return None

ignored_topics = [
    '/rosout_agg',
    '/rosout',
    # '/clock',
    '/clicked_point',

    '/initialpose',
    '/move_base_simple/goal',
    '/piksi/navsatfix_best_fix',
    # '/jackal_velocity_controller/odom',
    '/microstrain/imu/data',
    '/microstrain/mag',

    # '/velodyne_points',

    # '/zed2i/zed_node/depth/depth_registered',
    # '/zed2i/zed_node/left/image_rect_color',
    # '/zed2i/zed_node/left/camera_info',
    # '/zed2i/zed_node/imu/data',
    # '/zed2i/zed_node/right/camera_info',
    # '/zed2i/zed_node/depth/camera_info',
    '/zed2i/zed_node/right/image_rect_color',
    '/zed2i/zed_node/confidence/confidence_map',
    '/zed2i/zed_node/imu/mag',
    '/zed2i/zed_node/pose',

    ## lidar upsample
    '/transformed_odom',
    '/transformed_point_cloud',
    '/cumulative_point_cloud',
    # '/cumulative_origin_point_cloud',

    ## main
    '/islam/vlp_filtered_pointcloud',
    '/islam/pg_rgb',
    # '/islam/pg_depth',
    '/islam/zed_pointcloud',
    '/islam/vlp_depth',
    '/islam/pg_fused_pointcloud',\
    '/islam/pg_odom',
    '/islam/pg_camera_info',
    '/islam/vlp_pts',
    '/islam/vlp_odom',
    '/islam/zed_original_pointcloud',
    '/islam/vlp_debug_pointcloud',
]

discovered_topics = get_all_topic_types()
rospy.loginfo("Found topics and their message classes:\n")
topics = []

for topic, msg_type in discovered_topics:
    if topic in ignored_topics:
        continue

    msg_class = import_msg_class(msg_type)

    if not has_header_field(msg_class):
        rospy.logwarn(f"{topic}: [No header filed on topic]")
        continue

    if msg_class:
        rospy.loginfo(f"{topic}: {msg_class}")
        topics.append((topic, msg_class))
    else:
        rospy.logerr(f"{topic}: [Failed to import {msg_type}]")

# %% CSV logging setup
log_dir = os.path.expanduser("./logs")  # Log to user's home directory
os.makedirs(log_dir, exist_ok=True)
log_filename = os.path.join(log_dir, f"topic_stats_{time.strftime('%Y%m%d_%H%M%S')}.csv")
log_file = open(log_filename, 'w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow([
    'Timestamp',
    *[topic for topic, _ in topics],
    'total_processing_time_ms',
    'Timestamp_Variance_sec',
])  # CSV Header

rospy.loginfo("CSV log file generated")


# %%
last_time = rospy.get_rostime().now()

def synchronized_callback(*msgs):
    global last_time
    now = rospy.get_rostime().now()
    rospy.loginfo(f"synchronized_callback: {(rospy.get_rostime().now() - last_time).to_sec()}")
    last_time = now

    global csv_writer, log_file

    # Record start time for the entire zed_callback processing
    total_start_time = time.time()

    timestamps = []
    for msg in msgs:
        timestamps.append(msg.header.stamp.to_sec())

    total_end_time = time.time()
    total_processing_time_ms = (total_end_time - total_start_time) * 1000

    # Log data to CSV
    csv_writer.writerow([
        time.time(),  # System timestamp
        *timestamps,
        total_processing_time_ms,
        np.var(timestamps),
    ])
    log_file.flush()  # Ensure data is written to disk immediately


subscribers = []
for topic, msg_class in topics:
    subscribers.append(message_filters.Subscriber(topic, msg_class))
    rospy.loginfo(f"Subscriber added: {topic}: {msg_class}")

ats = message_filters.ApproximateTimeSynchronizer(
    subscribers,
    queue_size=10,  # Adjust as needed
    slop=20.0
)

# Register the synchronized callback
ats.registerCallback(synchronized_callback)

# %%
rospy.spin()
