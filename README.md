# LIDAR Upsample

![cumulative-rviz.png](assets/cumulative-rviz.png)

Current dependencies:

- ROS Noetic
- Ubuntu 20.04
- Python 3.8.10

## Bags

The bag files are collected from two different datasets with similar ROS topics. One of them is CitrusFarm and
the other one is SensorSuiteV2 dataset.

```mermaid\
graph TD
    Datasets ---> CitrusFarmDataset;
    Datasets ---> SensorSuiteV2Dataset;
```

- [2024-10-01-19-26-26.bag](../../DS/2024-10-01-19-26-26.bag)
- [2023-05-08-20-25-52.bag](../../DS/2023-05-08-20-25-52.bag)

## Run

```bash
roscore
```

### SensorSuiteV2 Dataset

```bash
source /opt/ros/noetic/setup.bash
rosbag play /home/joseph/Development/DS/2023-05-08-20-25-52.bag -l
```

```bash
rosbag play /home/joseph/Development/DS/2023-05-08-20-25-52.bag -l -s 55 --rate 0.1
```

### CitrusFarm Dataset

> :info: `--clock` flag is passed to be able to use the simulation time on the ros nodes. Otherwise, the
`ApproximateTimeSynchronizer` could not work fine when rosbag restarts

```bash
rosbag play -l -s 10 -u 10 --clock \
~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/base_2023-07-18-14-26-48_0.bag \
~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/zed_2023-07-18-14-26-49_0.bag \
~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/odom_2023-07-18-14-26-48.bag
```

> :warning: To make all the topics synchronized, we have to make the reply rate 0.1 (etc.). Otherwise, there will be
> delay between some topics.

```bash
rosbag play -l -s 10 -u 10 -r 0.1 --clock \
~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/base_2023-07-18-14-26-48_0.bag \
~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/zed_2023-07-18-14-26-49_0.bag \
~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/odom_2023-07-18-14-26-48.bag
```

## Run Topic Transformer

```bash
./transformer_lidar.sh
```

## Run Algorithm

```bash
source /opt/ros/noetic/setup.bash
python3 lidar_upsample.py
```

```bash
source /opt/ros/noetic/setup.bash
python3 meas_topic_stats.py
```

```bash
cd /home/joseph/PycharmProjects/citrusFarmSFITC/
python3 main.py
```

## RViz

```bash
source /opt/ros/noetic/setup.bash
rviz -d /media/joseph/Development/GitHub/lidar_upsample/rviz.rviz
```