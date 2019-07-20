
#ifndef DATABASE_H
#define DATABASE_H

// Original
#include <common.h>

namespace three_view_sfm {

class Database {
 public:
  Database();

  // Cloud Point
  bool IsCloudPointPresent(int view_idx0, int feature_idx0, int view_idx1,
                           int feature_idx1);

  CloudPoint &GetCloudPoint(int cp_idx);

  CloudPoint &AddCloudPoint(int view_idx0, int feature_idx0, int view_idx1,
                            int feature_idx1, cv::Point3d &pt);

  CloudPoint &AddCloudPointViewConnection(int view_idx0, int feature_idx0,
                                          int view_idx1, int feature_idx1);

  std::map<int, CloudPoint> &GetCloudPoints();

  // Two View
  bool IsTwoViewPresent(int view_idx0, int view_idx1);

  TwoView &GetTwoView(int view_idx0, int view_idx1);

  TwoView &AddTwoView(int view_idx0, int view_idx1);

  std::map<std::pair<int, int>, TwoView> &GetTwoViews();

  // View
  bool IsViewPresent(int view_idx);

  ViewData &GetView(int view_idx);

  ViewData &AddView(const std::string &path);

  std::map<int, ViewData> &GetViews();

  const std::map<int, ViewData> &GetViews() const;

  void SetCamIntrinsics(const CamIntrinsics &cam_intr);

  const CamIntrinsics &GetCamIntrincics();

  Status status;

 private:
  CamIntrinsics m_cam_intr;
  std::map<int, ViewData> views;
  std::map<std::pair<int, int>, TwoView> m_two_views;
  std::map<int, CloudPoint> m_cloud_points;
  std::map<std::string, int> m_img_path_idx;

  int m_next_view_idx;
  int m_next_cp_idx;
};

}  // namespace three_view_sfm

#endif  // DATABASE_H