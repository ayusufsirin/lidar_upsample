# LIDAR Upsample

Current dependencies:

- ROS Noetic
- Ubuntu 20.04
- Python 3.8.10

## Bags

- [2024-10-01-19-26-26.bag](../../DS/2024-10-01-19-26-26.bag)
- [2023-05-08-20-25-52.bag](../../DS/2023-05-08-20-25-52.bag)

## Run

```bash
docker compose up
```

```bash
source /opt/ros/noetic/setup.bash
rosbag play /home/joseph/Development/DS/2023-05-08-20-25-52.bag -l
rosbag play /home/joseph/Development/DS/2023-05-08-20-25-52.bag -l -s 55 --rate 0.1
```

```bash
source /opt/ros/noetic/setup.bash
rviz -d /media/joseph/Development/GitHub/lidar_upsample/rviz.rviz
```

```bash
./transformer_lidar.sh
```

```bash
source /opt/ros/noetic/setup.bash
python3 /media/joseph/Development/GitHub/lidar_upsample/lidar_upsample.py
```
