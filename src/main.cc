
// System
#include <iostream>

// Gflags
#include <gflags/gflags.h>

// Glog
#include <glog/logging.h>

// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>

using namespace std;

DEFINE_string(first_pic, "./data/0003.jpg", "");
DEFINE_string(second_pic, "./data/0005.jpg", "");
DEFINE_string(third_pic, "./data/0007.jpg", "");
DEFINE_double(vis_scale, 0.5, "");

void ScaleAndShowImage(const double scale, const cv::Mat &img,
                       const int wait_time) {
  cv::Mat show_img;
  cv::resize(img, show_img, cv::Size(), FLAGS_vis_scale, FLAGS_vis_scale);
  cv::imshow("Image", show_img);
  cv::waitKey(wait_time);
}

void SystemSetup(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = 1;
  FLAGS_stderrthreshold = google::GLOG_INFO;
  google::InitGoogleLogging(argv[0]);
}

void PrepareData() {}

void ExtractSIFTFeature() {
  cv::Mat image;
  cv::Mat descriptors;
  vector<cv::KeyPoint> key_points;
  cv::xfeatures2d::SiftDescriptorExtractor extractor;
  extractor.detectAndCompute(image, cv::Mat(), key_points, descriptors);
}

void MatchFeature() {}

void ComputeFundamentalMatrix() {}

void GeneratePointCloud() {}

void ComputePoseViaPNP() {}

void VisualizeResult() {}

int main(int argc, char **argv) {

  // 0. System Setup
  SystemSetup(argc, argv);

  // 1. Preparation
  PrepareData();

  // X. Feature Extraction
  ExtractSIFTFeature();

  // X. Feature Matching
  MatchFeature();

  // X. Compute Fundamental Matrix and Decide Scale.
  ComputeFundamentalMatrix();

  // X. Generate Point Cloud from Epipolar Geometry.
  GeneratePointCloud();

  // X. Compute Pose Via PNP
  ComputePoseViaPNP();

  // X. Add Third View via PNP.
  GeneratePointCloud();

  // X. Final Result Visualization.
  cv::Mat mat = cv::imread(FLAGS_first_pic, cv::IMREAD_UNCHANGED);
  ScaleAndShowImage(FLAGS_vis_scale, mat, 0);

  return 0;
}