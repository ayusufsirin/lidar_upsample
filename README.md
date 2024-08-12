source /opt/ros/noetic/setup.bash
cd /home/joseph/Development/DS
rosbag play 2023-05-08-20-25-52.bag -l

source /opt/ros/noetic/setup.bash
rviz -d /media/joseph/Development/GitHub/lidar_upsample/rviz.rviz

./transformer_lidar.sh