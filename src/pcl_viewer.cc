
// Self Header
#include <pcl_viewer.h>

// STL
#include <string>
#include <thread>

// PCL
#include <pcl/ros/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

// Original
#include <common.h>

using namespace std;

namespace {

using namespace three_view_sfm;

float CAM_SCALE_COEFF = 0.25;
float VIEWING_RAY_SCALE_COEFF = 3.0;
int CAM_POLYGON[18] = {0, 1, 2, 0, 3, 1, 0, 4, 3, 0, 2, 4, 3, 1, 4, 2, 4, 1};
cv::Scalar CAM_COLOR(255, 0, 0);

pcl::PointXYZRGB EigenToPointXYZRGB(Eigen::Vector3d v, Eigen::Vector3d rgb) {
  pcl::PointXYZRGB p(rgb[0], rgb[1], rgb[2]);
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

Eigen::Matrix<float, 6, 1> EigenToEigen(Eigen::Vector3d v,
                                        Eigen::Vector3d rgb) {
  return (Eigen::Matrix<float, 6, 1>() << v[0], v[1], v[2], rgb[0], rgb[1],
          rgb[2])
      .finished();
}

vector<Eigen::Matrix<float, 6, 1>> ToVector(
    const Eigen::Matrix<float, 6, 1>& p1,
    const Eigen::Matrix<float, 6, 1>& p2) {
  std::vector<Eigen::Matrix<float, 6, 1>> v(2);
  v[0] = p1;
  v[1] = p2;
  return v;
}

void ConvertCloudPointsToPclPointCloud(
    const vector<CloudPoint>& cloud_points,
    pcl::PointCloud<pcl::PointXYZRGB>& pcl_point_cloud) {
  // X. Erase all points.
  pcl_point_cloud.clear();

  for (auto cp : cloud_points) {
    pcl::PointXYZRGB pcl_pnt(cp.rgb[2], cp.rgb[1], cp.rgb[0]);
    pcl_pnt.x = cp.pt.x;
    pcl_pnt.y = cp.pt.y;
    pcl_pnt.z = cp.pt.z;
    pcl_point_cloud.push_back(pcl_pnt);
  }
}

void ComposeCameraElement(
    const cv::Matx34d& pose, const std::string& name, float r, float g, float b,
    vector<std::pair<std::string, pcl::PolygonMesh>>& cam_meshes,
    vector<std::pair<std::string, std::vector<Eigen::Matrix<float, 6, 1>>>>&
        lines_to_draw,
    double s) {
  // Confirm name is not empty.
  assert(name.length() > 0);

  Eigen::Matrix3d R;
  Eigen::Vector3d T, T_trans;
  R << pose(0, 0), pose(0, 1), pose(0, 2), pose(1, 0), pose(1, 1), pose(1, 2),
      pose(2, 0), pose(2, 1), pose(2, 2);
  T << pose(0, 3), pose(1, 3), pose(2, 3);

  T_trans = -R.transpose() * T;
  Eigen::Vector3d v_right, v_up, v_forward;
  v_right = R.row(0).normalized() * s;
  v_up = -R.row(1).normalized() * s;
  v_forward = R.row(2).normalized() * s;
  Eigen::Vector3d rgb(r, g, b);

  // Polygon Mesh
  {
    pcl::PointCloud<pcl::PointXYZRGB> mesh_cld;
    mesh_cld.push_back(EigenToPointXYZRGB(T_trans, rgb));
    mesh_cld.push_back(EigenToPointXYZRGB(
        T_trans + v_forward + v_right / 2.0 + v_up / 2.0, rgb));
    mesh_cld.push_back(EigenToPointXYZRGB(
        T_trans + v_forward + v_right / 2.0 - v_up / 2.0, rgb));
    mesh_cld.push_back(EigenToPointXYZRGB(
        T_trans + v_forward - v_right / 2.0 + v_up / 2.0, rgb));
    mesh_cld.push_back(EigenToPointXYZRGB(
        T_trans + v_forward - v_right / 2.0 - v_up / 2.0, rgb));

    pcl::PolygonMesh pm;
    pm.polygons.resize(6);
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 3; j++) {
        pm.polygons[i].vertices.push_back(CAM_POLYGON[i * 3 + j]);
      }
    }
    pcl::toROSMsg(mesh_cld, pm.cloud);
    cam_meshes.push_back(std::make_pair(name, pm));
  }

  // Viewing Rays.
  {
    std::string line_name = name + "line";
    lines_to_draw.push_back(std::make_pair(
        line_name,
        ToVector(
            EigenToEigen(T_trans, rgb),
            EigenToEigen(T_trans + v_forward * VIEWING_RAY_SCALE_COEFF, rgb))));
  }
}

void ComposeCameraElements(
    const vector<pair<int, cv::Matx34d>>& cam_poses,
    vector<pair<string, pcl::PolygonMesh>>& cam_meshes,
    vector<pair<string, vector<Eigen::Matrix<float, 6, 1>>>>& lines_to_draw) {
  for (auto pair : cam_poses) {
    std::stringstream ss;
    ss << "Camera" << to_string(pair.first);
    ComposeCameraElement(pair.second, ss.str(), CAM_COLOR(0), CAM_COLOR(1),
                         CAM_COLOR(2), cam_meshes, lines_to_draw,
                         CAM_SCALE_COEFF);
  }
}

}  // namespace

namespace three_view_sfm {

class PCLViewerInternalStorage {
 public:
  PCLViewerInternalStorage()
      : m_p_vis_thread(nullptr),
        m_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
        m_window_name("Point Cloud Viewer"),
        m_quit(false),
        m_updated(false) {}

  // Vis Thread.
  std::unique_ptr<std::thread> m_p_vis_thread;

  // Shared Buffer.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pcl_point_cloud;
  vector<std::pair<std::string, pcl::PolygonMesh>> m_cam_meshes;
  vector<pair<string, vector<Eigen::Matrix<float, 6, 1>>>> m_lines_to_draw;

  // Various variables.
  string m_window_name;
  bool m_quit, m_updated;
};

struct PCLViewerHandler {
  static void KeyBoardEventHandler(
      const pcl::visualization::KeyboardEvent& event, void* PCLViewer_void) {
    PCLViewer* p_pcl_viewer = static_cast<PCLViewer*>(PCLViewer_void);
    std::string key = event.getKeySym();
    if (key == "Escape" && event.keyDown()) {
      p_pcl_viewer->m_intl->m_quit = true;
    }
  }
};

PCLViewer::PCLViewer(const string& window_name)
    : m_intl(new PCLViewerInternalStorage()) {
  m_intl->m_window_name = window_name;
}

PCLViewer::~PCLViewer() {}

void PCLViewer::Update(const vector<CloudPoint>& point_cloud,
                       const vector<pair<int, cv::Matx34d>>& cameras) {
  LOG(INFO) << "Num of Points : " << point_cloud.size();
  ConvertCloudPointsToPclPointCloud(point_cloud, *m_intl->m_pcl_point_cloud);
  ComposeCameraElements(cameras, m_intl->m_cam_meshes, m_intl->m_lines_to_draw);
  m_intl->m_updated = true;
}

void PCLViewer::RunVisualizationAsync() {
  m_intl->m_p_vis_thread.reset(
      new std::thread(&PCLViewer::RunVisualization, this));
}

void PCLViewer::WaitVisThread() { m_intl->m_p_vis_thread->join(); }

void PCLViewer::RunVisualization() {
  // X. Create Viewer
  pcl::visualization::PCLVisualizer visualizer(m_intl->m_window_name);

  visualizer.registerKeyboardCallback(PCLViewerHandler::KeyBoardEventHandler,
                                      static_cast<void*>(this));

  while (!visualizer.wasStopped()) {
    if (m_intl->m_quit) {
      break;
    }

    if (m_intl->m_updated) {
      // X. Remove all points drawn already.
      visualizer.removePointCloud("Orig");
      visualizer.addPointCloud(m_intl->m_pcl_point_cloud, "Orig");

      for (int i = 0; i < m_intl->m_cam_meshes.size(); i++) {
        visualizer.removeShape(m_intl->m_cam_meshes[i].first);
        visualizer.addPolygonMesh(m_intl->m_cam_meshes[i].second,
                                  m_intl->m_cam_meshes[i].first);
      }
      m_intl->m_cam_meshes.clear();
      m_intl->m_updated = false;
    }
    visualizer.spinOnce();
  }
}

}  // namespace three_view_sfm