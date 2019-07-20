
// STL
#include <map>
#include <memory>
#include <vector>

// Original
#include <common.h>

namespace three_view_sfm {

struct PCLViewerInternalStorage;
struct PCLViewerHandler;
class PCLViewer {
 public:
  PCLViewer(const std::string& window_name);

  ~PCLViewer();

  virtual void Update(const std::vector<CloudPoint>& point_cloud,
                      const std::vector<std::pair<int, cv::Matx34d>>& cameras);

  void RunVisualizationAsync();

  void WaitVisThread();

  void RunVisualization();

 private:
  std::unique_ptr<PCLViewerInternalStorage> m_intl;

  friend struct PCLViewerHandler;
};

}  // namespace three_view_sfm