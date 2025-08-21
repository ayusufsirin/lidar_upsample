#!/bin/bash

frame_id="odom90"

rosrun tf2_ros static_transform_publisher 0 0 0 0 1.57079632679 0 odom $frame_id &

rosrun topic_tools transform /pg/rtabmap/odom /pg/rtabmap/odom90 nav_msgs/Odometry 'nav_msgs.msg.Odometry(
    header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="'$frame_id'"),
    child_frame_id=m.child_frame_id,
    pose=geometry_msgs.msg.PoseWithCovariance(
        pose=geometry_msgs.msg.Pose(
            position=m.pose.pose.position,
            orientation=m.pose.pose.orientation,
            ),
        covariance=m.pose.covariance),
    twist=m.twist)' --import nav_msgs std_msgs geometry_msgs &

rosrun topic_tools transform /zed/rtabmap/odom /zed/rtabmap/odom90 nav_msgs/Odometry 'nav_msgs.msg.Odometry(
    header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="'$frame_id'"),
    child_frame_id=m.child_frame_id,
    pose=geometry_msgs.msg.PoseWithCovariance(
        pose=geometry_msgs.msg.Pose(
            position=m.pose.pose.position,
            orientation=m.pose.pose.orientation,
            ),
        covariance=m.pose.covariance),
    twist=m.twist)' --import nav_msgs std_msgs geometry_msgs &

wait