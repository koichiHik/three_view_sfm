
// System
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <thread>
#include <vector>

// Boost
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

// Gflags
#include <gflags/gflags.h>

// Glog
#include <glog/logging.h>

// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

// Original
#include <bundle_adjust.h>
#include <cloud_point.h>
#include <common.h>
#include <database.h>
#include <feature.h>
#include <pcl_viewer.h>
#include <pnp_pose.h>
#include <two_view_pose.h>

using namespace std;
using namespace three_view_sfm;

DEFINE_string(directory, "./data/", "");
DEFINE_string(k_mat_file, "K.txt", "");
DEFINE_double(scale, 0.5, "");
DEFINE_int64(num_features, 0, "");

void RaiseAllFilesInDirectoryInternal(
    const std::string &dirpath, std::vector<std::string> &img_path_list,
    std::vector<std::string> &img_filename_list) {
  namespace fs = boost::filesystem;
  const fs::path path(dirpath);

  BOOST_FOREACH (const fs::path &p, std::make_pair(fs::directory_iterator(path),
                                                   fs::directory_iterator())) {
    if (!fs::is_directory(p)) {
      img_filename_list.push_back(p.filename().string());
      img_path_list.push_back(fs::absolute(p).string());
    }
  }

  std::sort(img_filename_list.begin(), img_filename_list.end());
  std::sort(img_path_list.begin(), img_path_list.end());
}

void ScaleAndShowImage(const int wait_time, const Database &db) {
  for (const auto &view : db.GetViews()) {
    cv::imshow("Image", view.second.image);
    cv::waitKey(wait_time);
  }
}

void SystemSetup(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = 1;
  FLAGS_stderrthreshold = google::GLOG_INFO;
  google::InitGoogleLogging(argv[0]);
}

void SetCameraIntrinsicsFromFile(const string &k_mat_path, const double scale,
                                 CamIntrinsics &cam_intr) {
  std::ifstream ifs(k_mat_path);
  CHECK(!ifs.fail()) << "Failed to load calib file : " << k_mat_path;

  cv::Matx33d &K = cam_intr.K;
  {
    std::string line;
    // 1st line
    getline(ifs, line);
    sscanf(line.c_str(), "%lf %lf %lf", &K(0, 0), &K(0, 1), &K(0, 2));

    // 2nd line
    getline(ifs, line);
    sscanf(line.c_str(), "%lf %lf %lf", &K(1, 0), &K(1, 1), &K(1, 2));

    // 2nd line
    getline(ifs, line);
    sscanf(line.c_str(), "%lf %lf %lf", &K(2, 0), &K(2, 1), &K(2, 2));
  }

  // X. Adjust Scale.
  {
    K(0, 0) *= scale;
    K(1, 1) *= scale;
    K(0, 2) *= scale;
    K(1, 2) *= scale;
  }

  cam_intr.Kinv = K.inv();
  cam_intr.dist_coeffs = cv::Matx41d(0, 0, 0, 0);
  LOG(INFO) << "Calibration file reading is done. : " << k_mat_path << endl
            << K;
}

void RaiseAllImagesInDirectory(const std::string &dirpath,
                               std::vector<std::string> &img_path_list,
                               const std::vector<std::string> &exts) {
  std::vector<std::string> tmp_img_path_list, tmp_file_name_list;
  RaiseAllFilesInDirectoryInternal(dirpath, tmp_img_path_list,
                                   tmp_file_name_list);

  for (int i = 0; i < tmp_img_path_list.size(); i++) {
    std::string abs_path = tmp_img_path_list[i];
    std::string filename = tmp_file_name_list[i];
    for (auto ext : exts) {
      if (abs_path.find(ext) == abs_path.size() - ext.size()) {
        img_path_list.push_back(abs_path);
        break;
      }
    }
  }
}

void LoadData(const string &data_directory, const string &k_mat_file,
              const double scale, Database &database) {
  // Load K matrix.
  CamIntrinsics cam_intr;
  SetCameraIntrinsicsFromFile(data_directory + k_mat_file, scale, cam_intr);
  database.SetCamIntrinsics(cam_intr);
  std::vector<std::string> image_paths;
  RaiseAllImagesInDirectory(data_directory, image_paths,
                            vector<string>{".jpg"});

  // Load Image Data.
  for (int i = 0; i < image_paths.size(); i++) {
    string path = image_paths[i];
    LOG(INFO) << "Load image file : " << path;
    cv::Mat tmp = cv::imread(path, cv::IMREAD_UNCHANGED);
    CHECK(!tmp.empty()) << "Failed to load image file : " + path;
    ViewData &view = database.AddView(path);
    view.file_path = path;
    cv::resize(tmp, view.image, cv::Size(), scale, scale);
    cv::cvtColor(view.image, view.gray_image, cv::COLOR_BGR2GRAY);
  }
}

vector<CloudPoint> GenerateCloudPointVectorFromMap(
    const std::map<int, CloudPoint> &cp_map) {
  vector<CloudPoint> cp_vec;
  for (auto key_val : cp_map) {
    cp_vec.push_back(key_val.second);
  }
  return cp_vec;
}

vector<pair<int, cv::Matx34d>> GenerateRecoveredPoseVector(Database &database) {
  vector<pair<int, cv::Matx34d>> pose_vector;
  for (auto view_idx : database.status.pose_recovered_view) {
    pose_vector.push_back(make_pair(view_idx, database.GetView(view_idx).P));
  }
  return pose_vector;
}

int main(int argc, char **argv) {
  // Step 0. System Setup
  SystemSetup(argc, argv);

  // Step 1. Data Preparation
  Database db;
  LoadData(FLAGS_directory, FLAGS_k_mat_file, FLAGS_scale, db);

  // Step 2. Prepare PCL Viewer.
  PCLViewer pcl_viewer("Sfm Result");
  pcl_viewer.RunVisualizationAsync();

  // Step 3. Feature Extraction
  ExtractSIFTFeature(FLAGS_num_features, db);

  // Step 4. Feature Matching
  MatchFeature(db);

  // Step 5. Filter by Funcatmental Matrix Calculation.
  FilterMatchedFeatureViaGeometricConstraints(db);

  // Step 6. Visualize Matching Result
  VisualizeMatchingResult(0, db);

  // Step 7. Compute Fundamental Matrix and Decide Scale.
  StructureFromMotionViaTwoViewGeometry(db);

  // Step 8. Bundle Adjust
  BundleAdjust(db);

  // Step 9. Visualizew Two View Pose.
  pcl_viewer.Update(GenerateCloudPointVectorFromMap(db.GetCloudPoints()),
                    GenerateRecoveredPoseVector(db));

  // Step 10. Sleep for visualization.
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // Step 11. Loop for all remaining views.
  while (db.status.processed_view.size() != db.GetViews().size()) {
    // Step 11.1 Compute Pose Via PNP
    StructureFromMotionViaPNP(db);

    // Step 11.2 Bundle Adjust
    BundleAdjust(db);

    // Step 11.3 Visualize Generated Point Cloud and Camera Poses.
    pcl_viewer.Update(GenerateCloudPointVectorFromMap(db.GetCloudPoints()),
                      GenerateRecoveredPoseVector(db));

    // Step 11.4 Sleep for visualization.
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  // Step 12. Wait till thread joins.
  pcl_viewer.WaitVisThread();

  return 0;
}