
// Self Header
#include <pnp_pose.h>

// STL
#include <set>

// OpenCV
#include <opencv2/calib3d.hpp>

// Original
#include <database.h>
#include <triangulation h>

namespace three_view_sfm {

using namespace std;

bool ChooseBestMatchingViewWithCurrentPointCloud(
    Database &database, int &best_view_idx, vector<cv::Point2d> &best_2d_pnts,
    vector<cv::Point3d> &best_3d_pnts, int &best_score) {
  // X. Loop all views that are not processed, yet.
  best_score = -1;
  best_view_idx = -1;
  for (auto this_view : database.GetViews()) {
    // If already processed, skip.
    if (database.status.processed_view.find(this_view.first) !=
        database.status.processed_view.end()) {
      continue;
    }

    // X. Loop all processed view to see connection between 2d pnt and cp.
    set<int> existed_cp_idx;
    vector<cv::Point2d> corresp_2d_pnts;
    vector<cv::Point3d> corresp_3d_pnts;
    for (auto processed_view_idx : database.status.processed_view) {
      // X. Get Matched Points.
      pair<int, int> key = MakeKey(this_view.first, processed_view_idx);
      // const TwoView &two_view = database.two_views[key];
      const TwoView &two_view =
          database.GetTwoView(this_view.first, processed_view_idx);
      const vector<cv::DMatch> &matched_vector = two_view.matched_points;
      // database.two_views[key].matched_points;

      for (int i = 0; i < matched_vector.size(); i++) {
        if (!two_view.validity[i]) {
          continue;
        }

        const cv::DMatch &match = matched_vector[i];
        int this_view_pnt_idx = match.queryIdx;
        int processed_view_pnt_idx = match.trainIdx;

        int cp_idx =
            FindWithDefault(database.GetView(processed_view_idx).kpnt_cp_idx,
                            processed_view_pnt_idx, -1);

        if (cp_idx != -1) {
          existed_cp_idx.insert(cp_idx);
          corresp_2d_pnts.push_back(
              database.GetView(this_view.first).points[this_view_pnt_idx]);
          corresp_3d_pnts.push_back(database.GetCloudPoint(cp_idx).pt);
        }
      }
    }

    int this_score = existed_cp_idx.size();
    if (best_score < this_score) {
      best_view_idx = this_view.first;
      best_2d_pnts = corresp_2d_pnts;
      best_3d_pnts = corresp_3d_pnts;
      best_score = this_score;
    }
  }

  return true;
}

bool FindCameraMatrixViaPNP(const vector<cv::Point2d> &image_pnts,
                            const vector<cv::Point3d> &cloud_pnts,
                            const CamIntrinsics &cam_intr, cv::Matx34d &P) {
  CHECK(image_pnts.size() == cloud_pnts.size())
      << "Number of 2D points and 3D points are different.";

  cv::Matx31d rodrigues, T;

  bool result = cv::solvePnPRansac(cloud_pnts, image_pnts, cam_intr.K,
                                   cam_intr.dist_coeffs, rodrigues, T);
  CHECK(result) << "Failed at solvePnPRansac";

  cv::Matx33d R;
  cv::Rodrigues(rodrigues, R);
  cv::Matx34d _P(R(0, 0), R(0, 1), R(0, 2), T(0, 0), R(1, 0), R(1, 1), R(1, 2),
                 T(1, 0), R(2, 0), R(2, 1), R(2, 2), T(2, 0));

  P = _P;

  return result;
}

bool GenerateNewPointCloud(const int view_idx, Database &database) {
  std::vector<TriangulatedPoint> triangulated_pnts;

  for (auto adopted_view_idx : database.status.adopted_view) {
    const pair<int, int> key = MakeKey(view_idx, adopted_view_idx);
    const TwoView &two_view = database.GetTwoView(view_idx, adopted_view_idx);
    const vector<uint8_t> validity = two_view.validity;
    cv::Matx34d Porg = database.GetView(adopted_view_idx).P;
    cv::Matx34d P1 = database.GetView(view_idx).P;

    for (int i = 0; i < two_view.matched_points.size(); i++) {
      const cv::DMatch &match = two_view.matched_points[i];

      if (validity[i] == 0) {
        continue;
      }

      int this_view_pnt_idx = match.queryIdx;
      int adopted_view_pnt_idx = match.trainIdx;

      // Check if this point has been alrady reconstructed.
      bool cp_present = database.IsCloudPointPresent(
          view_idx, this_view_pnt_idx, adopted_view_idx, adopted_view_pnt_idx);

      if (!cp_present) {
        cv::Point2f point0 =
            database.GetView(adopted_view_idx).points[adopted_view_pnt_idx];
        cv::Point2f point1 =
            database.GetView(view_idx).points[this_view_pnt_idx];
        pair<cv::Point3d, double> triangulated_pnt_with_err;
        TriangulatePoint(database.GetCamIntrincics().K, Porg, P1, point0,
                         point1, triangulated_pnt_with_err);
        CloudPoint &cp = database.AddCloudPoint(
            view_idx, this_view_pnt_idx, adopted_view_idx, adopted_view_pnt_idx,
            triangulated_pnt_with_err.first);
      } else {
        database.AddCloudPointViewConnection(view_idx, this_view_pnt_idx,
                                             adopted_view_idx,
                                             adopted_view_pnt_idx);
      }
    }

    LOG(INFO) << "Size of Cloud Points : " << database.GetCloudPoints().size();
  }

  return true;
}

void StructureFromMotionViaPNP(Database &database) {
  // X. Select view that mathces the current point cloud the best.
  int best_view_idx, best_score;
  vector<cv::Point2d> matched_2d_points;
  vector<cv::Point3d> matched_3d_points;
  set<int> not_recovered_2d_pnts;
  CHECK(ChooseBestMatchingViewWithCurrentPointCloud(
      database, best_view_idx, matched_2d_points, matched_3d_points,
      best_score))
      << "Can not find best matching view with current cloud.";

  // X. Status Update as Processed View.
  database.status.processed_view.insert(best_view_idx);

  LOG(INFO) << "Next Selected View : "
            << database.GetView(best_view_idx).file_path
            << ", Score : " << best_score;

  // X. Compute Pose with respect to current point cloud.
  CHECK(FindCameraMatrixViaPNP(matched_2d_points, matched_3d_points,
                               database.GetCamIntrincics(),
                               database.GetView(best_view_idx).P))
      << "Failed at FindCameraMatrixViaPNP";

  // X. Status Update as Processed View.
  database.status.pose_recovered_view.insert(best_view_idx);

  // X. Generate new point cloud.
  CHECK(GenerateNewPointCloud(best_view_idx, database)) << "";

  // X. Status Update as Adopted View.
  database.status.adopted_view.insert(best_view_idx);

  return;
}

}  // namespace three_view_sfm