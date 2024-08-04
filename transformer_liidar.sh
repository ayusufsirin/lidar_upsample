#!/bin/bash

source /opt/ros/noetic/setup.bash

# Function to clean up background processes
cleanup() {
    echo "Killing all background processes..."
    kill $(jobs -p)
    exit 0
}

# Trap SIGINT (Ctrl+C) and call cleanup
trap cleanup SIGINT

frame_id="map"  # common frame_id
topic_base="islam"

export vlp_pts_s="/velodyne_points"
export vlp_pts_p="/"$topic_base"/vlp_pts"

export vlp_path_s="/path"
export vlp_path_p="/"$topic_base"/vlp_path"

export vlp_odom_s="/odom"
export vlp_odom_p="/"$topic_base"/vlp_odom"

rosrun topic_tools transform $vlp_pts_s $vlp_pts_p sensor_msgs/PointCloud2 'sensor_msgs.msg.PointCloud2(header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="'$frame_id'"), data=m.data, height=m.height, width=m.width, fields=m.fields, is_bigendian=m.is_bigendian, point_step=m.point_step, row_step=m.row_step, is_dense=m.is_dense)' --import std_msgs sensor_msgs&

rosrun topic_tools transform $vlp_path_s $vlp_path_p nav_msgs/Path 'nav_msgs.msg.Path(header=std_msgs.msg.Header(seq=m.header.seq, stamp=m.header.stamp, frame_id="'$frame_id'"), poses=m.poses)' --import nav_msgs std_msgs&

rosrun topic_tools transform $vlp_odom_s $vlp_odom_p nav_msgs/Odometry 'nav_msgs.msg.Odometry(
    header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="'$frame_id'"),
    child_frame_id=m.child_frame_id,
    pose=geometry_msgs.msg.PoseWithCovariance(
        pose=geometry_msgs.msg.Pose(
            position=m.pose.pose.position,
            orientation=geometry_msgs.msg.Quaternion(
                x=m.pose.pose.orientation.x * 0.7071068 - m.pose.pose.orientation.z * 0.7071068,
                y=m.pose.pose.orientation.y * 0.7071068 - m.pose.pose.orientation.w * 0.7071068,
                z=m.pose.pose.orientation.z * 0.7071068 + m.pose.pose.orientation.x * 0.7071068,
                w=m.pose.pose.orientation.w * 0.7071068 + m.pose.pose.orientation.y * 0.7071068)),
        covariance=m.pose.covariance),
    twist=m.twist)' --import nav_msgs std_msgs geometry_msgs&

rosrun tf2_ros static_transform_publisher -35 -65 40 0.0 0.0 0.0 1.0 map loam_link&
rosrun tf2_ros static_transform_publisher 0 0 0 0.3535534 0.3535534 0.1464466 0.8535534 loam_link rot&

# Wait for background processes to finish
wait
