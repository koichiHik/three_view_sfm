project(simple_sfm)
cmake_minimum_required(VERSION 2.8)

# Use C++ 14
set(CMAKE_CXX_STANDARD 14)

# Output Path
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

# Necessary Libraries.
find_package(Eigen3 REQUIRED CONFIG PATHS ${Eigen3_DIR} NO_DEFAULT_PATH)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED CONFIG PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)
find_package(VTK REQUIRED)
find_package(Boost COMPONENTS filesystem program_options)

# Header Inclusion.
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Compile Executable.
add_executable(three_view_sfm
  src/main.cc
  src/feature.cc
  src/pnp_pose.cc
  src/triangulation.cc
  src/cloud_point.cc
  src/pcl_viewer.cc
  src/database.cc
  src/two_view_pose.cc)

# Link.
target_link_libraries(three_view_sfm
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)