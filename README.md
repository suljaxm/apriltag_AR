apriltag_AR
==========
apriltag_AR based on AprilTag 3.  This repository contains the visualization of camera pose and  landmark,  and compares `cv::solvePnP` and `apriltag_detector_detect`  to obtain [R, t].

---

​                            <img src=".\tmp\1.png" style="zoom:35%;" />            <img src=".\tmp\2.png" style="zoom:35%;" />

`apriltag_detector_detect`:

​                            <img src=".\tmp\tag1.png" style="zoom:35%;" />             <img src=".\tmp\tag2.png" style="zoom:35%;" />

`cv::solvePnP`:

​                            <img src=".\tmp\pnp1.png" style="zoom:35%;" />             <img src=".\tmp\pnp2.png" style="zoom:35%;" />



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

- [ ] g2o
- [ ] apriltag + orbslam

Flexible Layouts
================

You can generate your own tag families using [AprilTag-Generation](https://github.com/AprilRobotics/apriltag-generation), or directly use in my fold `./tag36h11`.

<img src=".\tag36h11\tag36h11_0.png" style="zoom:35%;" />

Papers
======

AprilTag is the subject of the following papers.

[AprilTag: A robust and flexible visual fiducial system](https://april.eecs.umich.edu/papers/details.php?name=olson2011tags)

[AprilTag 2: Efficient and robust fiducial detection](https://april.eecs.umich.edu/papers/details.php?name=wang2016iros)

[Flexible Layouts for Fiducial Tags](https://april.eecs.umich.edu/papers/details.php?name=krogius2019iros)