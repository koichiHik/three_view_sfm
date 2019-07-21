
// Self Header
#include <feature.h>

// System
#include <algorithm>

// Glog
#include <glog/logging.h>

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

namespace {

const double FMAT_REPERR_SCALING = 0.0006;
const double FMAT_RANSAC_CONFIDENCE = 0.9800;

template <typename T1, typename T2, typename ConvertFunctor>
void ConvertVector(vector<T1> &src, vector<T2> &dst, ConvertFunctor func) {
  dst.resize(src.size());
  std::transform(src.begin(), src.end(), dst.begin(), func);
}

}  // namespace

namespace three_view_sfm {

void ExtractSIFTFeature(const int num_features, Database &database) {
  cv::Ptr<cv::xfeatures2d::SIFT> extractor =
      cv::xfeatures2d::SIFT::create(num_features);

  for (auto &view : database.GetViews()) {
    LOG(INFO) << "Extracting SIFT features for image : " +
                     view.second.file_path;
    extractor->detectAndCompute(view.second.gray_image, cv::Mat(),
                                view.second.key_points,
                                view.second.descriptors);
    auto convertFunctor = [](cv::KeyPoint &pnt) -> cv::Point2f {
      return pnt.pt;
    };
    ConvertVector<cv::KeyPoint, cv::Point2f>(
        view.second.key_points, view.second.points, convertFunctor);
  }
}

vector<cv::DMatch> FlipMatches(vector<cv::DMatch> &matches) {
  vector<cv::DMatch> flip_matches = matches;
  for (auto &match : flip_matches) {
    std::swap(match.queryIdx, match.trainIdx);
  }
  return flip_matches;
}

void MatchFeature(Database &database) {
  cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2, true);
  int image_num = database.GetViews().size();
  for (int query_idx = 0; query_idx < image_num; query_idx++) {
    for (int train_idx = 0; train_idx < image_num; train_idx++) {
      if (query_idx == train_idx) {
        continue;
      }

      ViewData &query_view = database.GetView(query_idx);
      ViewData &train_view = database.GetView(train_idx);
      vector<cv::DMatch> match_result;
      matcher->match(query_view.descriptors, train_view.descriptors,
                     match_result);
      database.AddTwoView(query_idx, train_idx).matched_points = match_result;
    }
  }
}

bool FilterMatchedFeatureViaGeometricConstraints(Database &database) {
  int image_num = database.GetViews().size();
  for (int query_idx = 0; query_idx < image_num; query_idx++) {
    for (int train_idx = 0; train_idx < image_num; train_idx++) {
      if (query_idx == train_idx) {
        continue;
      }

      ViewData &query_view = database.GetView(query_idx);
      ViewData &train_view = database.GetView(train_idx);
      // pair<int, int> key = MakeKey(query_idx, train_idx);
      const vector<cv::DMatch> &matched_vector =
          database.GetTwoView(query_idx, train_idx).matched_points;
      vector<cv::Point2f> aligned_points0, aligned_points1;

      LOG(INFO) << "query_idx : " << query_idx << ", train_idx : " << train_idx;

      AlignMatchVector(matched_vector, query_view.points, train_view.points,
                       aligned_points0, aligned_points1);
      vector<uint8_t> status;
      double min_val, max_val;
      cv::minMaxLoc(aligned_points1, &min_val, &max_val);
      cv::Mat F = cv::findFundamentalMat(
          aligned_points0, aligned_points1, cv::FM_RANSAC,
          max_val * FMAT_REPERR_SCALING, FMAT_RANSAC_CONFIDENCE, status);
      database.GetTwoView(query_idx, train_idx).F = F;
      database.GetTwoView(query_idx, train_idx).validity = status;

      // X. Simple Statistics.
      LOG(INFO) << "Fundamental matrix calculation is done.";
      LOG(INFO) << query_view.file_path << " vs " << train_view.file_path
                << ", Num points matched : " << matched_vector.size()
                << ", Num points inliers : " << cv::countNonZero(status);
    }
  }
  return true;
}

void VisualizeMatchingResult(const int wait_time, Database &database) {
  int image_num = database.GetViews().size();
  for (int query_idx = 0; query_idx < image_num - 1; query_idx++) {
    for (int train_idx = query_idx + 1; train_idx < image_num; train_idx++) {
      cv::Mat matched_img;
      const ViewData &query_view = database.GetView(query_idx);
      const ViewData &train_view = database.GetView(train_idx);
      const TwoView &two_view = database.GetTwoView(query_idx, train_idx);

      vector<char> status;
      status.resize(two_view.validity.size());
      std::transform(two_view.validity.begin(), two_view.validity.end(),
                     status.begin(),
                     [](uint8_t a) -> char { return static_cast<char>(a); });

      cv::drawMatches(query_view.image, query_view.key_points, train_view.image,
                      train_view.key_points, two_view.matched_points,
                      matched_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                      status);
      LOG(INFO) << "The match between " << query_idx << " vs " << train_idx;
      LOG(INFO) << "The number of match " << two_view.matched_points.size();
      cv::imshow("Matched Image", matched_img);
      cv::waitKey(wait_time);
    }
  }
  cv::destroyWindow("Matched Image");
}

}  // namespace three_view_sfm