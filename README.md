# Point clouds viewing, filtering and saving

Code changes are based on [iai_kinect2](https://github.com/code-iai/iai_kinect2), new functions are added such as hand region detection, filtering and saving point clouds.

The final code includes training and real-time classification will be open source after publication.
## Dependencies
* ROS Hydro/Indigo/Kinetic
* OpenCV (2.4.x, using the one from the official Ubuntu repositories is recommended)
* PCL (1.7.x, using the one from the official Ubuntu repositories is recommended)
* Eigen (optional, but recommended)
* OpenCL (optional, but recommended)
* libfreenect2 (>= v0.2.0, for stability checkout the latest stable release)
## Install
Follow the steps in [iai_kinect2](https://github.com/code-iai/iai_kinect2)

## Assumptions

A few assumptions have been made:

* The camera is supposed to be static.
* Hand is supposed in front of the body.

## Usage

Clone the project:
```git
cd ~/catkin_ws/src/
git clone https://github.com/Xinjiann/point_clouds_saver.git
cd point_clouds_saver
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
```
Run:
```git
Roscore
```
New terminal, run:
```git
roslaunch kinect2_bridge kinect2_bridge
```
If you want to segment the hand region, run:
```git
rosrun point_clouds_saver saver sd filter
```
Else, run:
```git
rosrun point_clouds_saver saver sd
```
To save a sequence of point clouds, press 'b' to start and 'e' to stop.

<img src="https://github.com/Xinjiann/Point-clouds-saver/blob/main/img/new.png" width = "450" height = "300" />


## Hand segmentation

### PassThrough filter to remove background
Filter out the points whose value is not in the given value range in the specified dimension direction.

<img src="https://github.com/Xinjiann/Point-clouds-saver/blob/main/img/first_.png" width = "440" height = "255" align=center/>
<img src="https://github.com/Xinjiann/Point-clouds-saver/blob/main/img/second_.png" width = "440" height = "255" align=center/>


### statisticalOutlierRemoval to remove noise

Assuming that the average distance between all points in the point cloud and its nearest k neighbor points meets the Gaussian distribution, then a distance threshold can be determined according to the mean and variance. When the average distance between a point and its nearest k points is greater than this threshold, Determine the point as an outlier and remove it.

First, traverse the point cloud and calculate the average distance between each point and its nearest k neighbors. Then calculate the mean μ and standard deviation σ of all average distances, the distance threshold dmax can be expressed as 

dmax=μ+α×σ 

Where α is a constant, which depends on the number of neighbor points. Finally, traverse again Point cloud, remove the points whose average distance from k neighbor points is greater than dmax.

### PassThrough filter to remove body region

The noise points has been removed in the last step, so the closest point from camera shoud be the one in hand region, then apply PassThrough filter again with a smaller threshold from the closest point to remove the body region.

<img src="https://github.com/Xinjiann/Point-clouds-saver/blob/main/img/third_.png" width = "440" height = "255" align=center/>

### PassThrough filter to remove arm region
Traverse the point cloud and find the point with largest y, and apply PassThrough filter to remove the arm region.
<img src="https://github.com/Xinjiann/Point-clouds-saver/blob/main/img/hand.png" width = "440" height = "255" align=center/>

### VoxelGrid filter downsampling point cloud

Use voxelized grid method to achieve downsampling. Reduce the number of points and save the shape characteristics of the point cloud at the same time. 

PCL realised the VoxelGrid class by creating a three-dimensional voxel grid through the input point cloud data, and use the center of gravity of all points in each voxel to approximate other points in the voxel, so that all points in the voxel are finally expressed with a center of gravity.

<img src="https://github.com/Xinjiann/Point-clouds-saver/blob/main/img/final.png" width = "440" height = "255" align=center/>
