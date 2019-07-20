
#ifndef COMMON_H
#define COMMON_H

// System
#include <map>
#include <set>
#include <string>

// Glog
#include <glog/logging.h>

// OpenCV
#include <opencv2/core.hpp>

namespace three_view_sfm {

template <typename T>
std::pair<T, T> MakeKey(T elem0, T elem1) {
  T first_elem, second_elem;
  if (elem0 < elem1) {
    first_elem = elem0;
    second_elem = elem1;
  } else {
    first_elem = elem1;
    second_elem = elem0;
  }
  // return std::make_pair(first_elem, second_elem);
  return std::make_pair(elem0, elem1);
}

template <typename Collection>
const typename Collection::value_type::second_type &FindOrDie(
    const Collection &collection,
    const typename Collection::value_type::first_type &key) {
  typename Collection::const_iterator cit = collection.find(key);
  CHECK(cit != collection.cend()) << "Map key not found : " << key;
  return cit->second;
}

template <typename Collection>
typename Collection::value_type::second_type &FindOrDie(
    Collection &collection,
    const typename Collection::value_type::first_type &key) {
  typename Collection::iterator it = collection.find(key);
  CHECK(it != collection.end()) << "Map key not found : " << key;
  return it->second;
}

template <typename Collection>
const typename Collection::value_type::second_type &FindWithDefault(
    const Collection &collection,
    const typename Collection::value_type::first_type &key,
    const typename Collection::value_type::second_type &value) {
  typename Collection::const_iterator cit = collection.find(key);
  if (cit == collection.cend()) {
    return value;
  }
  return cit->second;
}

template <typename T>
bool AlignMatchVector(const std::vector<cv::DMatch> &match_vector,
                      const std::vector<T> &org_points1,
                      const std::vector<T> &org_points2,
                      std::vector<T> &aligned_points1,
                      std::vector<T> &aligned_points2) {
  aligned_points1.clear();
  aligned_points2.clear();
  int pnt_num1 = org_points1.size();
  int pnt_num2 = org_points2.size();

  for (int i = 0; i < match_vector.size(); i++) {
    const cv::DMatch &d_match = match_vector[i];
    int q_idx = d_match.queryIdx;
    int t_idx = d_match.trainIdx;
    if (q_idx < 0 || t_idx < 0) {
      continue;
    }
    CHECK(q_idx < pnt_num1)
        << "Given index is bigger than the number of points detected. q_idx : "
        << q_idx << ", pnt_num1 : " << pnt_num1;
    CHECK(t_idx < pnt_num2)
        << "Given index is bigger than the number of points detected. t_idx : "
        << t_idx << ", pnt_num2 : " << pnt_num2;
    aligned_points1.push_back(org_points1[q_idx]);
    aligned_points2.push_back(org_points2[t_idx]);
  }

  return true;
}

template <typename T>
bool AlignMatchVector(const std::vector<cv::DMatch> &match_vector,
                      const std::vector<uint8_t> &validity,
                      const std::vector<T> &org_points1,
                      const std::vector<T> &org_points2,
                      std::vector<cv::DMatch> &valid_match,
                      std::vector<T> &aligned_points1,
                      std::vector<T> &aligned_points2) {
  valid_match.clear();
  aligned_points1.clear();
  aligned_points2.clear();
  int pnt_num1 = org_points1.size();
  int pnt_num2 = org_points2.size();

  for (int i = 0; i < match_vector.size(); i++) {
    const cv::DMatch &d_match = match_vector[i];
    int q_idx = d_match.queryIdx;
    int t_idx = d_match.trainIdx;
    if ((q_idx < 0 || t_idx < 0) || validity[i] == 0) {
      continue;
    }
    CHECK(q_idx < pnt_num1)
        << "Given index is bigger than the number of points detected.";
    CHECK(t_idx < pnt_num2)
        << "Given index is bigger than the number of points detected.";
    valid_match.push_back(d_match);
    aligned_points1.push_back(org_points1[q_idx]);
    aligned_points2.push_back(org_points2[t_idx]);
  }

  return true;
}

struct ViewData {
  std::string file_path;
  cv::Mat image;
  cv::Mat gray_image;
  std::vector<cv::KeyPoint> key_points;
  std::vector<cv::Point2f> points;
  cv::Mat descriptors;
  cv::Matx34d P;
  std::map<int, int> kpnt_cp_idx;
};

struct TwoView {
  std::vector<cv::DMatch> matched_points;
  std::vector<cv::DMatch> inlier_matched_points;
  std::vector<uint8_t> validity;
  cv::Matx33d F;
  cv::Matx33d E;
};

struct TriangulatedPoint {
  cv::Point3d pt;
  std::pair<int, int> view_pnt_idx0;
  std::pair<int, int> view_pnt_idx1;
};

struct CloudPoint {
  CloudPoint() {}

  CloudPoint(cv::Point3d _pt)
      : pt(_pt), rgb(0, 0, 0), rgb_sum(0, 0, 0), view_feature_idx() {}
  cv::Point3d pt;
  cv::Vec3b rgb;
  cv::Vec3i rgb_sum;
  std::map<int, int> view_feature_idx;
};

struct Status {
  std::set<int> processed_view;
  std::set<int> pose_recovered_view;
  std::set<int> adopted_view;
};

struct CamIntrinsics {
  cv::Matx33d K;
  cv::Matx33d Kinv;
  cv::Matx41d dist_coeffs;
};

inline bool UpdateCloudPoint(CloudPoint &cp, int view_idx, int feature_idx,
                             cv::Vec3b &rgb) {
  cp.view_feature_idx[view_idx] = feature_idx;
  cp.rgb_sum += rgb;
  double num = cp.view_feature_idx.size();
  cp.rgb =
      cv::Vec3b(cp.rgb_sum(0) / num, cp.rgb_sum(1) / num, cp.rgb_sum(2) / num);
}

}  // namespace three_view_sfm

#endif