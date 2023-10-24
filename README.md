# Ring FALS Normal Estimator
A real-time normal estimator for spinning LiDAR.
## Related Works

1. [LOG-LIO](https://github.com/tiev-tongji/LOG-LIO): A LiDAR-inertial Odometry with Efficient Local Geometric Information Estimation

**Related video:**  [Ring FALS](https://youtu.be/cxTLywI7X7M)

![normals.jpg](fig%2Fnormals.jpg)

## How to use
Tested under Ubuntu 16.04 with opencv3.3.1 and Ubuntu 18.04 with opencv3.2.0. 
Output point cloud with the topic "cloud_normal".

```angular2html
mkdir -p ws_ring_fals/src
cd ws_ring_fals/src
git clone https://github.com/tiev-tongji/RingFalsNormal
```
set _**OpenCV_DIR**_ in the CMakeLists.txt to your local path, and please compile the  _**opencv-contrib**_ module in advance.
```
cd .. & catkin_make
source devel/setup.bash
```

### run with our pre-built lookup table
For the [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) dataset
```
roslaunch ring_fals normal_m2dgr.launch 
```
For the [NTU VIRAL](https://github.com/ntu-aris/ntu_viral_dataset) dataset
```
roslaunch ring_fals normal_viral.launch 
```
So far, we have only provided launch files for [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) and [NTU VIRAL](https://github.com/ntu-aris/ntu_viral_dataset).

### build a lookup table for a spinning LiDAR
Set the parameters in the _**yaml**_ file for the LiDAR carefully.
**_compute_table_** must be set to true.
```angular2html
    compute_table: true               # true: compute only the lookup table
    ring_table_dir: "/table_dir"      # lookup table path, read or write
```
The lookup table will be saved to the path specified by  **_ring_table_dir_** upon ros shutdown.
```
roslaunch ring_fals *.launch
rosbag play *.bag
```
Finally, run with your lookup table
```
roslaunch ring_fals *.launch 
```
