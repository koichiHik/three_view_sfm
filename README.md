# three_view_sfm

This is a simple implementation of Structure from Motion. The code is intended to be used for learning purposes.

Little bit of code walk through is written in my blog.
http://daily-tech.hatenablog.com/entry/2019/07/21/230633

### Reference
The book "Mastering OpenCV with Practical Computer Vision Projects" and its source code "Ch4) Exploring Structure from Motion using OpenCV, by Roy Shilkrot" is referenced to understand algorithmic and implementation details.
https://github.com/MasteringOpenCV/code

### Prerequisite
You need the following libraries for successful build.

OpenCV
Eigen
GLOG
GFLAG
Boost

### Command
#### Download from Github & Build
git clone git@github.com:koichiHik/three_view_sfm.git three_view_sfm
cd three_view_sfm
sh build.sh

#### Download Test Data from Github
cd ..
git clone git@github.com:openMVG/SfM_quality_evaluation.git SfM_quality_evaluation

#### Run command
##### 1. Run sfm for fountain-P11 data.
cd three_view_sfm
./build/bin/three_view_sfm --directory=../SfM_quality_evaluation/Benchmarking_Camera_Calibration_2008/fountain-P11/images/
