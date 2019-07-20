

// Self Header
#include <two_view_pose.h>

// Glog
#include <glog/logging.h>

// OpenCV
#include <opencv2/calib3d.hpp>

// Original
#include <common.h>
#include <triangulation h>

namespace {

using namespace std;
using namespace three_view_sfm;

const double FMAT_INLINER_NUM_THRESH = 100;
const double EMAT_ZERO_THRESH = 0.00001;
const double TRIANGULATE_FRONT_PNT_RATIO = 0.8;

bool ChooseAndSetUpSeedTwoView(Database &database, pair<int, int> &pair_idx) {
  pair<int, int> max_pair_idx = make_pair(0, 1);
  const TwoView &two_view = database.GetTwoView(0, 1);
  int max_num_inliers = -1;
  for (auto two_view : database.GetTwoViews()) {
    int num_inliers = cv::countNonZero(two_view.second.validity);

    if (max_num_inliers < num_inliers) {
      max_pair_idx = two_view.first;
      max_num_inliers = num_inliers;
    }
  }

  // X. Setup origin
  database.GetView(max_pair_idx.first).P =
      cv::Matx34d(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  pair_idx = max_pair_idx;

  return true;
}

bool ComputeEssentialMatrix(const pair<int, int> &pair_idx,
                            Database &database) {
  TwoView &two_view = database.GetTwoView(pair_idx.first, pair_idx.second);

  // X. Calculate Essential Mat from F and K.
  two_view.E = database.GetCamIntrincics().K.t() * two_view.F *
               database.GetCamIntrincics().K;

  // X. Validity Check. E is square matrix of rank 2.
  double val = std::abs(cv::determinant(two_view.E));
  CHECK(val < EMAT_ZERO_THRESH)
      << "Quality of E mat is bad. Its determinant is : " << val
      << " that is more than threshold.";

  return true;
}

cv::Matx34d CreateCameraMatrixFromRT(const cv::Matx33d &R,
                                     const cv::Matx31d &T) {
  return cv::Matx34d(R(0, 0), R(0, 1), R(0, 2), T(0, 0), R(1, 0), R(1, 1),
                     R(1, 2), T(1, 0), R(2, 0), R(2, 1), R(2, 2), T(2, 0));
}

bool DecomposeEssentialMatrixToRT(const pair<int, int> &pair_idx,
                                  Database &database,
                                  vector<cv::Matx34d> &four_camera_matrices) {
  // X. SVD decomposition of E.
  cv::SVD svd(database.GetTwoView(pair_idx.first, pair_idx.second).E);
  bool flip = cv::determinant(svd.u * svd.vt) < -0.9;
  if (flip) {
    svd(-database.GetTwoView(pair_idx.first, pair_idx.second).E);
  }

  // X. 2 Non-Zero Singular Values have to be almost same.
  double singular_value_ratio =
      std::abs(svd.w.at<double>(0) / svd.w.at<double>(1));
  CHECK(0.7 < singular_value_ratio && singular_value_ratio < 1.4)
      << "Two non-zero singular values are far apart. Theoretically they have "
         "to be the same. : "
      << singular_value_ratio;

  // X. Rotation Matrix Calculation.
  cv::Matx33d W(0, -1, 0, 1, 0, 0, 0, 0, 1);
  cv::Matx33d R1 = cv::Mat(svd.u * cv::Mat(W) * svd.vt);
  cv::Matx33d R2 = cv::Mat(svd.u * cv::Mat(W.t()) * svd.vt);

  LOG(INFO) << "R1 : " << cv::determinant(R1)
            << ", R2 : " << cv::determinant(R2);

  // X. Translation Vector Calculation.
  cv::Matx31d T1 = cv::Mat(svd.u.col(2));
  cv::Matx31d T2 = cv::Mat(-svd.u.col(2));

  // X. Create 4 Possible Camera Matrices.
  four_camera_matrices.push_back(CreateCameraMatrixFromRT(R1, T1));
  four_camera_matrices.push_back(CreateCameraMatrixFromRT(R1, T2));
  four_camera_matrices.push_back(CreateCameraMatrixFromRT(R2, T1));
  four_camera_matrices.push_back(CreateCameraMatrixFromRT(R2, T2));

  // X. Status Update as Pose Recovered Camera.
  database.status.pose_recovered_view.insert(pair_idx.first);
  database.status.pose_recovered_view.insert(pair_idx.second);

  return true;
}

bool CheckProjectedPointsInFrontOfCamera(
    const vector<cv::Point3d> &triangulated_pnts, const cv::Matx34d &P,
    vector<uint8_t> &status, double thresh_point_ratio_in_front) {
  cv::Matx44d P4x4 = cv::Matx44d::eye();
  for (int i = 0; i < 12; i++) {
    P4x4.val[i] = P.val[i];
  }
  vector<cv::Point3d> projected_points;
  cv::perspectiveTransform(triangulated_pnts, projected_points, P4x4);

  // If projected point is in front of camera, valid.
  status.resize(projected_points.size());
  for (int i = 0; i < projected_points.size(); i++) {
    status[i] = projected_points[i].z > 0.0 ? 1 : 0;
  }

  // Compute valid ratio.
  int valid_count = cv::countNonZero(status);

  // X. Log
  LOG(INFO) << "Total Point # : " << status.size() << ", Valid Point # : "
            << "valid_count # : " << valid_count << ", Front Point Ratio : "
            << (valid_count / (double)projected_points.size());

  return thresh_point_ratio_in_front <
         (valid_count / (double)projected_points.size());
}

bool Triangulate(const pair<int, int> &pair_idx,
                 const vector<cv::Matx34d> four_possible_camera_matrices,
                 Database &database) {
  // X. Align Matched Key Point.
  const ViewData &view0 = database.GetView(pair_idx.first);
  const ViewData &view1 = database.GetView(pair_idx.second);
  const TwoView &two_view =
      database.GetTwoView(pair_idx.first, pair_idx.second);
  vector<cv::DMatch> valid_match;
  vector<cv::Point2f> aligned_points0, aligned_points1;
  vector<pair<cv::Point3d, double>> triangulated_pnt_with_err;
  AlignMatchVector(two_view.matched_points, two_view.validity, view0.points,
                   view1.points, valid_match, aligned_points0, aligned_points1);

  // X. Try all possible camera matrices.
  for (int i = 0; i < four_possible_camera_matrices.size(); i++) {
    LOG(INFO) << "Check R, T Configufation : Try " << i << endl
              << "P = " << endl
              << four_possible_camera_matrices[i] << endl
              << endl;
    cv::Matx34d Porg = view0.P;
    cv::Matx34d P1 = four_possible_camera_matrices[i];
    bool result = TriangulatePoints(database.GetCamIntrincics().K, Porg, P1,
                                    aligned_points0, aligned_points1,
                                    triangulated_pnt_with_err);
    vector<cv::Point3d> triangulated_pnts(triangulated_pnt_with_err.size());
    vector<uint8_t> status;
    transform(triangulated_pnt_with_err.cbegin(),
              triangulated_pnt_with_err.cend(), triangulated_pnts.begin(),
              [](const pair<cv::Point3d, double> &pnt_w_err) -> cv::Point3d {
                return pnt_w_err.first;
              });
    result = result &&
             CheckProjectedPointsInFrontOfCamera(triangulated_pnts, P1, status,
                                                 TRIANGULATE_FRONT_PNT_RATIO);

    if (result) {
      LOG(INFO) << "Triangulation Succedded with Configuration # " << i;

      database.GetView(pair_idx.second).P = P1;
      for (int i = 0; i < triangulated_pnt_with_err.size(); i++) {
        if (status[i] == 0) {
          continue;
        }

        bool cp_exist = database.IsCloudPointPresent(
            pair_idx.first, valid_match[i].queryIdx, pair_idx.second,
            valid_match[i].trainIdx);
        if (!cp_exist) {
          database.AddCloudPoint(pair_idx.first, valid_match[i].queryIdx,
                                 pair_idx.second, valid_match[i].trainIdx,
                                 triangulated_pnt_with_err[i].first);
        } else {
          database.AddCloudPointViewConnection(
              pair_idx.first, valid_match[i].queryIdx, pair_idx.second,
              valid_match[i].trainIdx);
        }
      }

      // X. Status Update as Adoped View.
      database.status.adopted_view.insert(pair_idx.first);
      database.status.adopted_view.insert(pair_idx.second);

      break;
    } else if (i == four_possible_camera_matrices.size() - 1) {
      LOG(FATAL) << "Failed to triangulate points.";
    }
  }

  return true;
}

bool ComputeCameraMatrix(Database &database, pair<int, int> &choosed_pair_idx,
                         vector<cv::Matx34d> &four_possible_camera_matrices) {
  // X. Choose Seed Two View.
  CHECK(ChooseAndSetUpSeedTwoView(database, choosed_pair_idx));
  LOG(INFO) << "Seed two view is : "
            << database.GetView(choosed_pair_idx.first).file_path << " and "
            << database.GetView(choosed_pair_idx.second).file_path;

  // X. Compute Essential Matrix.
  CHECK(ComputeEssentialMatrix(choosed_pair_idx, database))
      << "Failed to compute essential matrix.";

  // X. Decompose E mat to RT.
  CHECK(DecomposeEssentialMatrixToRT(choosed_pair_idx, database,
                                     four_possible_camera_matrices))
      << "Failed to decompose essential matrix.";

  return true;
}

bool TriangulateWithPossibleConfigurations(
    pair<int, int> &choosed_pair_idx, Database &database,
    vector<cv::Matx34d> &four_possible_camera_matrices) {
  // X. Triangulate.
  CHECK(Triangulate(choosed_pair_idx, four_possible_camera_matrices, database))
      << "Failed to triangulate.";
  return true;
}

}  // namespace

namespace three_view_sfm {

void StructureFromMotionViaTwoViewGeometry(Database &database) {
  pair<int, int> choosed_pair_idx;
  vector<cv::Matx34d> four_possible_camera_matrices;
  // X. Compute Camera Matrix.
  CHECK(ComputeCameraMatrix(database, choosed_pair_idx,
                            four_possible_camera_matrices))
      << "Failed to compute camera matrix.";

  // X. Status Update as Processed View.
  database.status.processed_view.insert(choosed_pair_idx.first);
  database.status.processed_view.insert(choosed_pair_idx.second);
  database.status.pose_recovered_view.insert(choosed_pair_idx.first);
  database.status.pose_recovered_view.insert(choosed_pair_idx.second);

  // X. Triangulate with All Possible Configuraions.
  CHECK(TriangulateWithPossibleConfigurations(choosed_pair_idx, database,
                                              four_possible_camera_matrices))
      << "Failed to Triangulate.";

  database.status.adopted_view.insert(choosed_pair_idx.first);
  database.status.adopted_view.insert(choosed_pair_idx.second);
}

}  // namespace three_view_sfm