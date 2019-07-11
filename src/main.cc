
// System
#include <iostream>

// Gflags
#include <gflags/gflags.h>

// Glog
#include <glog/logging.h>

// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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

int main(int argc, char **argv) {

  // 0. System Setup
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = 1;
  FLAGS_stderrthreshold = google::GLOG_INFO;
  google::InitGoogleLogging(argv[0]);

  cv::Mat mat = cv::imread(FLAGS_first_pic, cv::IMREAD_UNCHANGED);
  ScaleAndShowImage(FLAGS_vis_scale, mat, 0);

  return 0;
}