# multisensor-SLAM

## Software installation

Software dependencies and installation procedure are specified in the "Setup"-folder.

## Download KITTI odometry data

```bash
$ cd ~/Downloads/
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_calib.zip
$ unzip 2011_10_03_drive_0027_sync.zip -d ~/Videos/
$ unzip 2011_10_03_calib.zip -d ~/Videos/
```


### Create rosbag from KITTI data

```bash
$ cd ~/Videos/dataset
$ kitti2bag -t 2011_09_26 -r 0002 raw_synced .
```

## Setup

```bash
$ git clone https://github.com/tshellum/multisensor-SLAM.git
$ cd multisensor-SLAM
$ git submodule update --init --recursive
```


## Usage

Modify the config files to fit the dataset that is to be used.

```bash
$ roscore
$ catkin build
$ roslaunch stereo_frontend kitti.launch
$ rosbag play /path/to/rosbag
```


## Visualization

### Plotting the optimization graph

Run the system following the instructions above. Convert the created .dot file to a visual file using the following command.

```bash
dot -Tps graph.dot -o graph.ps
```

### Plotting the estimated trajectory compared to GNSS data

```bash
$ rosrun rpg_trajectory_evaluation analyze_trajectory_single.py results/ --recalculate_errors
```