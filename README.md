# Ring FALS Normal Estimator
A real-time normal estimator for spinning LiDAR.
## Related Works

1. [LOG-LIO](https://github.com/tiev-tongji/RingFalsNormal): A LiDAR-inertial Odometry with Efficient Local Geometric Information Estimation

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

```angular2html
cd .. & catkin_make

source devel/setup.bash
roslaunch ring_fals normal_m2dgr.launch 
```

So far, we have only provided luanch files for [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) and [NTU VIRAL](https://github.com/ntu-aris/ntu_viral_dataset), a more detailed readme is coming soon!

