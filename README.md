apriltag_AR
==========
apriltag_AR based on AprilTag 3.  This repository contains the visualization of camera pose and  landmark,  and compares `cv::solvePnP` and `apriltag_detector_detect`  to obtain [R, t].

---

​                  <img width="320" height="240" src=".\tmp\1.png" style="zoom:100%;" />            <img width="320" height="240" src=".\tmp\2.png" style="zoom:100%;" />

`apriltag_detector_detect`:

​                  <img width="320" height="240" src=".\tmp\tag1.png" style="zoom:100%;" />            <img width="320" height="240" src=".\tmp\tag2.png" style="zoom:100%;" />

`cv::solvePnP`:

​                  <img width="320" height="240" src=".\tmp\pnp1.png" style="zoom:100%;" />             <img width="320" height="240" src=".\tmp\pnp2.png" style="zoom:100%;" />

`void Draw`:  

<img width="640" height="400" src=".\tmp\pointclound" style="zoom:100%;" />

  ( blue: landmarks ,  red: camera poses)

Usage
=====
[User Guide](https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide)

Dependencies
=======

- Eigen
- OpenCV >= 3.1
- Sophus 
- G2O
- pangolin

### Compile

```
mkdir build
cd build
cmake ..
make
```

### run

```
./opencv_demo
```

## TODO

- [x] visualization 

- [x] g2o
- [ ] apriltag + orbslam

Flexible Layouts
================

You can generate your own tag families using [AprilTag-Generation](https://github.com/AprilRobotics/apriltag-generation), or directly use in my fold `./tag36h11`.

<div align=center><img width="200" height="250" src=".\tag36h11\tag36h11_0.png" style="zoom:100%;" /> </div>
Papers
======

AprilTag is the subject of the following papers.

[AprilTag: A robust and flexible visual fiducial system](https://april.eecs.umich.edu/papers/details.php?name=olson2011tags)

[AprilTag 2: Efficient and robust fiducial detection](https://april.eecs.umich.edu/papers/details.php?name=wang2016iros)

[Flexible Layouts for Fiducial Tags](https://april.eecs.umich.edu/papers/details.php?name=krogius2019iros)