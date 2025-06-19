# %% Performance
import csv
import importlib
import logging
import os
import time

import message_filters
import numpy as np
import rospy

# %%
rospy.init_node('sf', anonymous=True)
rospy.set_param('/rosgraph/log_level', logging.DEBUG)

rospy.loginfo("Node initialized")


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
    '/clock',
]

discovered_topics = get_all_topic_types()
rospy.loginfo("Found topics and their message classes:\n")
topics = []

for topic, msg_type in discovered_topics:
    if topic in ignored_topics:
        continue

    msg_class = import_msg_class(msg_type)
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
