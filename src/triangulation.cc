
// Self Header
#include <triangulation h>

// Glog
#include <glog/logging.h>

using namespace std;

namespace {

const double TRIANGULATE_WEIGHT_MINIMUM_UPDATE = 1.0e-7;

cv::Point3d NormalizePoint(const cv::Point2f &pnt, const cv::Matx33d &Kinv) {
  // Homogeneous & Normalize.
  cv::Matx31d homogeneous(pnt.x, pnt.y, 1.0);
  cv::Matx31d norm_pnt = Kinv * homogeneous;
  return cv::Point3d(norm_pnt(0), norm_pnt(1), norm_pnt(2));
}

void BuildInhomogeneousEqnSystemForTriangulation(
    const cv::Point3d &pnt0, const cv::Matx34d &P0, const cv::Point3d &pnt1,
    const cv::Matx34d &P1, const double w0, const double w1, cv::Matx43d &A,
    cv::Matx41d &B) {
  // Matrix A
  cv::Matx43d A_(
      (pnt0.x * P0(2, 0) - P0(0, 0)) / w0, (pnt0.x * P0(2, 1) - P0(0, 1)) / w0,
      (pnt0.x * P0(2, 2) - P0(0, 2)) / w0, (pnt0.y * P0(2, 0) - P0(1, 0)) / w0,
      (pnt0.y * P0(2, 1) - P0(1, 1)) / w0, (pnt0.y * P0(2, 2) - P0(1, 2)) / w0,
      (pnt1.x * P1(2, 0) - P1(0, 0)) / w1, (pnt1.x * P1(2, 1) - P1(0, 1)) / w1,
      (pnt1.x * P1(2, 2) - P1(0, 2)) / w1, (pnt1.y * P1(2, 0) - P1(1, 0)) / w1,
      (pnt1.y * P1(2, 1) - P1(1, 1)) / w1, (pnt1.y * P1(2, 2) - P1(1, 2)) / w1);

  // Matrix B
  cv::Matx41d B_(-(pnt0.x * P0(2, 3) - P0(0, 3)) / w0,
                 -(pnt0.y * P0(2, 3) - P0(1, 3)) / w0,
                 -(pnt1.x * P1(2, 3) - P1(0, 3)) / w1,
                 -(pnt1.y * P1(2, 3) - P1(1, 3)) / w1);

  A = A_;
  B = B_;
}

void SolveLinearEqn(cv::Matx43d &A, cv::Matx41d &B, cv::Matx41d &X) {
  cv::Matx31d tmpX;
  cv::solve(A, B, tmpX, cv::DECOMP_SVD);
  X(0) = tmpX(0);
  X(1) = tmpX(1);
  X(2) = tmpX(2);
  X(3) = 1.0;
}

cv::Matx41d IterativeLinearLSTriangulation(const cv::Point3d &pnt0,
                                           const cv::Matx34d &P0,
                                           const cv::Point3d &pnt1,
                                           const cv::Matx34d &P1) {
  // X. Build Linear Eqn System.
  double w0(1.0), w1(1.0);
  cv::Matx43d A;
  cv::Matx41d B, X;
  BuildInhomogeneousEqnSystemForTriangulation(pnt0, P0, pnt1, P1, w0, w1, A, B);
  SolveLinearEqn(A, B, X);

  // X. Iteratively Solve Linear Eqns.
  for (int i = 0; i < 10; i++) {
    // X. Weight Update.
    double p2x0 = (P0.row(2) * X)(0);
    double p2x1 = (P1.row(2) * X)(0);

    // X. Check Convergence.
    if (std::abs(w0 - p2x0) < TRIANGULATE_WEIGHT_MINIMUM_UPDATE &&
        std::abs(w1 - p2x1) < TRIANGULATE_WEIGHT_MINIMUM_UPDATE) {
      break;
    }

    // X. Weight Update and Solve
    w0 = p2x0;
    w1 = p2x1;
    BuildInhomogeneousEqnSystemForTriangulation(pnt0, P0, pnt1, P1, w0, w1, A,
                                                B);
    SolveLinearEqn(A, B, X);
  }

  return X;
}
}  // namespace

namespace three_view_sfm {

bool TriangulatePoint(const cv::Matx33d &K, const cv::Matx34d &Porigin,
                      const cv::Matx34d &P, const cv::Point2f &point0,
                      const cv::Point2f &point1,
                      pair<cv::Point3d, double> &triangulated_point_with_err) {
  // X. Preparation.
  cv::Matx34d KP = K * P;
  cv::Matx33d Kinv = K.inv();

  // X. Triangulation.
  cv::Point3d norm_pnt0 = NormalizePoint(point0, Kinv);
  cv::Point3d norm_pnt1 = NormalizePoint(point1, Kinv);
  cv::Matx41d X =
      IterativeLinearLSTriangulation(norm_pnt0, Porigin, norm_pnt1, P);

  // X. Compute Reprojection Error
  double reproj_err;
  {
    cv::Matx31d proj_pnt = KP * X;
    cv::Point2f normalized_proj_pnt(proj_pnt(0) / proj_pnt(2),
                                    proj_pnt(1) / proj_pnt(2));
    reproj_err = cv::norm(normalized_proj_pnt - point1);
    cv::Point3d pnt(X(0), X(1), X(2));
    triangulated_point_with_err = make_pair(pnt, reproj_err);
  }

  return true;
}

bool TriangulatePoints(
    const cv::Matx33d &K, const cv::Matx34d &Porigin, const cv::Matx34d &P,
    const vector<cv::Point2f> &aligned_points0,
    const vector<cv::Point2f> &aligned_points1,
    vector<pair<cv::Point3d, double>> &triangulated_points_with_err) {
  CHECK(aligned_points0.size() == aligned_points1.size())
      << "Number of points to be triangulated is different!";

  // X. Preparation.
  triangulated_points_with_err.clear();
  cv::Matx34d KP = K * P;
  cv::Matx33d Kinv = K.inv();

  // X. Triangulate All Points.
  vector<double> reproj_errors;
  for (int i = 0; i < aligned_points0.size(); i++) {
    // X. Triangulation.
    cv::Point3d norm_pnt0 = NormalizePoint(aligned_points0[i], Kinv);
    cv::Point3d norm_pnt1 = NormalizePoint(aligned_points1[i], Kinv);
    cv::Matx41d X =
        IterativeLinearLSTriangulation(norm_pnt0, Porigin, norm_pnt1, P);

    // X. Compute Reprojection Error
    {
      cv::Matx31d proj_pnt = KP * X;
      cv::Point2f normalized_proj_pnt(proj_pnt(0) / proj_pnt(2),
                                      proj_pnt(1) / proj_pnt(2));
      double reproj_err = cv::norm(normalized_proj_pnt - aligned_points1[i]);
      reproj_errors.push_back(reproj_err);
      cv::Point3d pnt(X(0), X(1), X(2));
      triangulated_points_with_err.push_back(make_pair(pnt, reproj_err));
    }
  }

  // X. Final Statistics.
  double sum =
      std::accumulate(reproj_errors.cbegin(), reproj_errors.cend(), 0.0);
  double mean = sum / reproj_errors.size();

  // X. Log
  LOG(INFO) << "Mean Reprojection Error : " << mean;

  return true;
}

}  // namespace three_view_sfm