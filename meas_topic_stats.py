# %% Performance
import csv
import logging
import os
import time

import message_filters
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, CameraInfo, Image

# %% Topics
topics = {
    '/velodyne_points': PointCloud2,
    '/zed2i/zed_node/left/camera_info': CameraInfo,
    '/islam/vlp_pts': PointCloud2,
    '/zed2i/zed_node/depth/depth_registered': Image,
    "/cumulative_origin_point_cloud": PointCloud2,
    '/zed2i/zed_node/left/image_rect_color': Image,
    '/zed2i/zed_node/depth/camera_info': CameraInfo,
    '/islam/vlp_odom': Odometry,
}

# %%
rospy.init_node('sf', anonymous=True)
rospy.set_param('/rosgraph/log_level', logging.DEBUG)

rospy.loginfo("Node initialized")

# %% CSV logging setup
log_dir = os.path.expanduser("./logs")  # Log to user's home directory
os.makedirs(log_dir, exist_ok=True)
log_filename = os.path.join(log_dir, f"topic_stats_{time.strftime('%Y%m%d_%H%M%S')}.csv")
log_file = open(log_filename, 'w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow([
    'Timestamp',
    *topics.keys(),
    'Timestamp_Variance_sec',
])  # CSV Header

rospy.loginfo("CSV log file generated")


# %%
def synchronized_callback(*msgs):
    rospy.loginfo("synchronized_callback")

    global csv_writer, log_file

    # Record start time for the entire zed_callback processing
    total_start_time = time.time()

    timestamps = []
    for msg in msgs:
        timestamps.append(msg.header.stamp.to_sec())

    # End timing for the entire zed_callback processing
    total_end_time = time.time()
    total_processing_time_ms = (total_end_time - total_start_time) * 1000

    # Log data to CSV
    csv_writer.writerow([
        time.time(),  # System timestamp
        *timestamps,
        np.var(timestamps),
    ])
    log_file.flush()  # Ensure data is written to disk immediately


subscribers = []
for topic, topic_type in topics.items():
    subscribers.append(message_filters.Subscriber(topic, topic_type))

ats = message_filters.ApproximateTimeSynchronizer(
    subscribers,
    queue_size=10,  # Adjust as needed
    slop=20.0
)

# Register the synchronized callback
ats.registerCallback(synchronized_callback)

# %%
rospy.spin()
