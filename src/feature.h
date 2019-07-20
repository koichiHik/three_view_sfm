
#ifndef FEATURE_H
#define FEATURE_H

// System
#include <iostream>
#include <map>
#include <vector>

// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

// Original
#include <common.h>
#include <database.h>

namespace three_view_sfm {

void ExtractSIFTFeature(const int num_features, Database &database);

std::vector<cv::DMatch> FlipMatches(std::vector<cv::DMatch> &matches);

void MatchFeature(Database &database);

bool FilterMatchedFeatureViaGeometricConstraints(Database &database);

void VisualizeMatchingResult(const int wait_time, Database &database);

}  // namespace three_view_sfm

#endif