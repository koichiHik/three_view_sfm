
// Self
#include <cloud_point.h>

#if 0

namespace three_view_sfm {

void ColorizeCloudPoint(const std::vector<int> &cp_updated_indices,
                        std::map<int, ViewData> &view_data_map,
                        std::map<int, CloudPoint> &cloud_point_map) {
  for (auto cp_idx : cp_updated_indices) {
    CloudPoint &cp = cloud_point_map[cp_idx];
    std::vector<cv::Vec3b> colors;
    for (auto view_feature : cp.view_feature_idx) {
      const ViewData &view = view_data_map[view_feature.first];
      const cv::Point2i pt = view.points[view_feature.second];
      colors.push_back(view.image.at<cv::Vec3b>());
    }
    cv::Scalar color = cv::mean(colors);
    cp.rgb = cv::Vec3b(color[0], color[1], color[2]);
  }
}

void GeneratePointCloud(const std::vector<TriangulatedPoint> &triangulated_pnts,
                        std::map<int, ViewData> &view_data_map,
                        std::map<int, CloudPoint> &cloud_point_map) {
  int next_idx = cloud_point_map.size();
  for (auto tri_pnt : triangulated_pnts) {
    // X. Existence Check
    // Check with view1.
    int cp_idx0, cp_idx1;
    {
      int view_idx0 = tri_pnt.view_pnt_idx0.first;
      int pnt_idx0 = tri_pnt.view_pnt_idx0.second;
      ViewData &view0 = FindOrDie(view_data_map, view_idx0);

      int view_idx1 = tri_pnt.view_pnt_idx1.first;
      int pnt_idx1 = tri_pnt.view_pnt_idx1.second;
      ViewData &view1 = FindOrDie(view_data_map, view_idx1);

      int cp_idx0 = FindWithDefault(view0.kpnt_cp_idx, pnt_idx0, -1);
      int cp_idx1 = FindWithDefault(view1.kpnt_cp_idx, pnt_idx1, -1);

      if (cp_idx0 == -1 && cp_idx1 == -1) {
        // Completely new.
        // New Cloudpoint Instance Generation.
        CloudPoint cp(tri_pnt.pt);
        cp.view_feature_idx[view_idx0] = pnt_idx0;
        cp.view_feature_idx[view_idx1] = pnt_idx1;
        cp.rgb_sum = view0.image.at<cv::Vec3b>(view0.points[pnt_idx0]);
        cp.rgb_sum += view1.image.at<cv::Vec3b>(view1.points[pnt_idx1]);
        double num = cp.view_feature_idx.size();
        cp.rgb = cv::Vec3b(cp.rgb_sum(0) / num, cp.rgb_sum(1) / num,
                           cp.rgb_sum(2) / num);

        // Update Cloud Point Database.
        cloud_point_map[next_idx] = cp;

        // Update View Database.
        view0.kpnt_cp_idx[pnt_idx0] = next_idx;
        view1.kpnt_cp_idx[pnt_idx1] = next_idx;

        next_idx++;

      } else if (cp_idx0 == -1 || cp_idx1 == -1) {
        // Already contained in one view.
        if (cp_idx0 == -1) {
          CloudPoint &cp = cloud_point_map[cp_idx1];
          cp.view_feature_idx[view_idx0] = pnt_idx0;
          cp.rgb_sum += view0.image.at<cv::Vec3b>(view0.points[pnt_idx0]);
          double num = cp.view_feature_idx.size();
          cp.rgb = cv::Vec3b(cp.rgb_sum(0) / num, cp.rgb_sum(1) / num,
                             cp.rgb_sum(2) / num);

          // Update View Database.
          view0.kpnt_cp_idx[pnt_idx0] = cp_idx1;
        } else {
          CloudPoint &cp = cloud_point_map[cp_idx0];
          cp.view_feature_idx[view_idx1] = pnt_idx1;
          cp.rgb_sum += view1.image.at<cv::Vec3b>(view1.points[pnt_idx1]);
          double num = cp.view_feature_idx.size();
          cp.rgb = cv::Vec3b(cp.rgb_sum(0) / num, cp.rgb_sum(1) / num,
                             cp.rgb_sum(2) / num);

          // Update View Database.
          view1.kpnt_cp_idx[pnt_idx1] = cp_idx0;
        }

      } else {
        LOG(FATAL) << "Both view already contains triangulated point. View : "
                   << view_idx0 << ", Point : " << pnt_idx0
                   << ", View : " << view_idx1 << ", Point : " << pnt_idx1;
      }
    }
  }
}

}  // namespace three_view_sfm

#endif