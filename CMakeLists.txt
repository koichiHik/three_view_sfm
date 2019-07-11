project(simple_sfm)
cmake_minimum_required(VERSION 2.8)

# Use C++ 14
set(CMAKE_CXX_STANDARD 14)

# Necessary Libraries.
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED CONFIG PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)
find_package(VTK REQUIRED)
find_package(CUDA REQUIRED)
find_package(Boost COMPONENTS filesystem program_options)

# All header file inclusion from top level directory.
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include Directories
add_executable(three_view_sfm
  src/main.cc)

