# multisensor-SLAM

## Software setup

Software dependencies and installation procedure are specified in the "Setup"-folder.

## Download KITTI odometry data

```bash
$ cd ~/Downloads/
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_color.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip
$ unzip data_odometry_gray.zip -d ~/Videos/
$ unzip data_odometry_color.zip -d ~/Videos/
$ unzip data_odometry_velodyne.zip -d ~/Videos/
$ unzip data_odometry_poses.zip -d ~/Videos/
$ unzip -o data_odometry_calib.zip -d ~/Videos/
```

### Create rosbag from KITTI data

```bash
$ cd ~/Videos/dataset
$ sudo kitti2bag -s 00 odom_gray .
# $ sudo kitti2bag odom -c gray -s 00
# $ kitti2bag -t 2011_09_26 -r 0002 raw_synced .
```

## Usage

TO BE WRITTEN

Modify the config files to fit the dataset.

```bash
$ roscore
$ catkin build
$ roslaunch package file.launch topic:=...
$ rosbag play /path/to/rosbag
$ rosrun rpg_trajectory_evaluation analyze_trajectory_single.py results/ --recalculate_errors
```
