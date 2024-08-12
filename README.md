# LIDAR Upsample

Current dependencies:

- ROS Noetic
- Ubuntu 20.04
- Python 3.8.10

## Run

```bash
source /opt/ros/noetic/setup.bash
cd /home/joseph/Development/DS
rosbag play 2023-05-08-20-25-52.bag -l
```

```bash
source /opt/ros/noetic/setup.bash
rviz -d /media/joseph/Development/GitHub/lidar_upsample/rviz.rviz
```

```bash
./transformer_lidar.sh
```
