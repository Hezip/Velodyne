# 32e-velodyne dual-mode driver

This is a driver about 32e velodyne **only for** dual mode.

## Getting Started

These instructions may help you to run this code in you local machine. If not, please contact me~

### Prerequisites

* [ros-drivers/velodyne](https://github.com/ros-drivers/velodyne) -- origion velodyne driver

### Installing

```
1. Copy the convert.cc/convert.h to ~/velodyne/velodyne_pointcloud/src/conversions
```
```
2. Copy the rawdata.cc to ~/velodyne/velodyne_pointcloud/src/lib
```
```
3. Copy the long.launch to ~/velodyne/velodyne_pointcloud/launch
```
```
4. Modify the long.launch for the group ns (default: **left_velodyne**)
```

## Running the tests

```
1. Record a data.bag of 32e velodyne dual mode (only record **~/packets** topic)
```
```
2. roslaunch velodyne_pointcloud long.launch
```
```
3. rosrun rviz rviz
```
```
4. velodyne_points ----> last return          velodyne_points_he ----> strongest return 
```

## Authors

* **He Zhenpeng**