#!/bin/bash

ROOT_3RD=~/workspace/3rdParty
CMAKE_BUILD_TYPE=Release
OpenCV_DIR_RELEASE=${ROOT_3RD}/opencv331/install/share/
OpenCV_DIR_DEBUG=${ROOT_3RD}/opencv331/installd/share/
Eigen3_DIR=${ROOT_3RD}/eigen337/install/share/eigen3/cmake/
CMAKE_MODULE_PATH=${ROOT_3RD}/pcl/install/share/pcl-1.7/Modules

if [ CMAKE_BUILD_TYPE="Release" ]; then
  OpenCV_DIR=${OpenCV_DIR_RELEASE}
else
  OpenCV_DIR=${OpenCV_DIR_DEBUG}
fi

if [ ! -e ./build ]; then
  mkdir build
fi
cd build

cmake \
  -D CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
  -D OpenCV_DIR=${OpenCV_DIR} \
  -D Eigen3_DIR=${Eigen3_DIR} \
  -D CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH} \
  ../

make

cd ../
