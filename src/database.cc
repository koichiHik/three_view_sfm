
// Self Header
#include <database.h>

using namespace std;

namespace three_view_sfm {

Database::Database() : m_next_view_idx(0), m_next_cp_idx(0) {}

std::map<int, CloudPoint> &Database::GetCloudPoints() { return m_cloud_points; }

CloudPoint &Database::GetCloudPoint(int cp_idx) {
  CHECK(m_cloud_points.find(cp_idx) != m_cloud_points.end())
      << "Cloud Point does not exist int the database. : " << cp_idx;
  return m_cloud_points[cp_idx];
}

bool Database::IsCloudPointPresent(int view_idx0, int feature_idx0,
                                   int view_idx1, int feature_idx1) {
  ViewData &view0 = views[view_idx0];
  ViewData &view1 = views[view_idx1];

  int cp_idx_from_view0 = FindWithDefault(view0.kpnt_cp_idx, feature_idx0, -1);
  int cp_idx_from_view1 = FindWithDefault(view1.kpnt_cp_idx, feature_idx1, -1);
  return cp_idx_from_view0 != -1 || cp_idx_from_view1 != -1;
}

CloudPoint &Database::AddCloudPoint(int view_idx0, int feature_idx0,
                                    int view_idx1, int feature_idx1,
                                    cv::Point3d &pt) {
  // X. Retrieve target views.
  ViewData &view0 = views[view_idx0];
  ViewData &view1 = views[view_idx1];
  int cp_idx_from_view0 = FindWithDefault(view0.kpnt_cp_idx, feature_idx0, -1);
  int cp_idx_from_view1 = FindWithDefault(view1.kpnt_cp_idx, feature_idx1, -1);

  CHECK(cp_idx_from_view0 == -1 && cp_idx_from_view1 == -1)
      << "This cloud point already exists.";

  int cur_cp_idx = m_next_cp_idx;
  m_next_cp_idx++;
  {
    // Completely New Case.
    CloudPoint cp(pt);
    UpdateCloudPoint(cp, view_idx0, feature_idx0,
                     view0.image.at<cv::Vec3b>(view0.points[feature_idx0]));
    view0.kpnt_cp_idx[feature_idx0] = cur_cp_idx;

    UpdateCloudPoint(cp, view_idx1, feature_idx1,
                     view1.image.at<cv::Vec3b>(view1.points[feature_idx1]));
    view1.kpnt_cp_idx[feature_idx1] = cur_cp_idx;

    // Add to Point Cloud.
    m_cloud_points[cur_cp_idx] = cp;
  }
  return m_cloud_points[cur_cp_idx];
}

CloudPoint &Database::AddCloudPointViewConnection(int view_idx0,
                                                  int feature_idx0,
                                                  int view_idx1,
                                                  int feature_idx1) {
  // X. Retrieve target views.
  ViewData &view0 = views[view_idx0];
  ViewData &view1 = views[view_idx1];
  int cp_idx_from_view0 = FindWithDefault(view0.kpnt_cp_idx, feature_idx0, -1);
  int cp_idx_from_view1 = FindWithDefault(view1.kpnt_cp_idx, feature_idx1, -1);

  CHECK(cp_idx_from_view0 != -1 || cp_idx_from_view1 != -1)
      << "This cloud point does not exist in the database.";

  int cp_idx = cp_idx_from_view0 == -1 ? cp_idx_from_view1 : cp_idx_from_view0;
  CloudPoint &cp = m_cloud_points[cp_idx];
  if (cp_idx_from_view0 == -1) {
    // Cp already exists.
    UpdateCloudPoint(cp, view_idx0, feature_idx0,
                     view0.image.at<cv::Vec3b>(view0.points[feature_idx0]));
    view0.kpnt_cp_idx[feature_idx0] = cp_idx_from_view1;
  } else if (cp_idx_from_view1 == -1) {
    // Cp already exists.
    UpdateCloudPoint(cp, view_idx1, feature_idx1,
                     view1.image.at<cv::Vec3b>(view1.points[feature_idx1]));
    view1.kpnt_cp_idx[feature_idx1] = cp_idx_from_view0;
  }
  return cp;
}

bool Database::IsTwoViewPresent(int view_idx0, int view_idx1) {
  pair<int, int> key = make_pair(view_idx0, view_idx1);
  return m_two_views.find(key) != m_two_views.end();
}

TwoView &Database::GetTwoView(int view_idx0, int view_idx1) {
  pair<int, int> key = make_pair(view_idx0, view_idx1);
  CHECK(m_two_views.find(key) != m_two_views.end())
      << "TwoView does not exists. : "
      << "view_idx0 : " << view_idx0 << "view_idx1 : " << view_idx1;
  return m_two_views[key];
}

std::map<std::pair<int, int>, TwoView> &Database::GetTwoViews() {
  return m_two_views;
}

TwoView &Database::AddTwoView(int view_idx0, int view_idx1) {
  pair<int, int> key = make_pair(view_idx0, view_idx1);

  CHECK(m_two_views.find(key) == m_two_views.end())
      << "TwoView already exists. : "
      << "view_idx0 : " << view_idx0 << "view_idx1 : " << view_idx1;
  m_two_views[key] = TwoView();
  return m_two_views[key];
}

bool Database::IsViewPresent(int view_idx) {
  return views.find(view_idx) != views.end();
}

ViewData &Database::GetView(int view_idx) {
  CHECK(views.find(view_idx) != views.end())
      << "View does not exist in the database. : " << view_idx;
  return views[view_idx];
}

ViewData &Database::AddView(const std::string &path) {
  CHECK(m_img_path_idx.find(path) == m_img_path_idx.end())
      << "Image already exists in database.";

  int cur_view_idx = m_next_view_idx;
  m_next_view_idx++;
  ViewData data;
  views[cur_view_idx] = data;
  return views[cur_view_idx];
}

std::map<int, ViewData> &Database::GetViews() { return views; }

const std::map<int, ViewData> &Database::GetViews() const { return views; }

void Database::SetCamIntrinsics(const CamIntrinsics &cam_intr) {
  this->m_cam_intr = cam_intr;
}

const CamIntrinsics &Database::GetCamIntrincics() { return m_cam_intr; }

}  // namespace three_view_sfm