
// STL
#include <map>
#include <vector>

// Original
#include <common.h>

namespace three_view_sfm {

void ColorizeCloudPoint(const std::vector<int> &cp_updated_indices,
                        std::map<int, ViewData> &view_data,
                        std::map<int, CloudPoint> &cloud_point_map);

void GeneratePointCloud(const std::vector<TriangulatedPoint> &new_cps,
                        std::map<int, ViewData> &view_data_map,
                        std::map<int, CloudPoint> &cloud_point_map);

}  // namespace three_view_sfm