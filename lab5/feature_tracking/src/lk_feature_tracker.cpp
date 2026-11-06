#include "lk_feature_tracker.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/ros.h>

LKFeatureTracker::LKFeatureTracker() = default;
LKFeatureTracker::~LKFeatureTracker() = default;

void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {
  if (frame.empty()) {
    ROS_WARN("Skipping an empty LK frame.");
    return;
  }

  cv::Mat gray;
  if (frame.channels() == 1) {
    gray = frame;
  } else {
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  }

  // Detect Harris corners afresh in each frame, then track the previous
  // frame's corners. This keeps long sequences from running out of tracks.
  std::vector<cv::Point2f> current_corners;
  cv::goodFeaturesToTrack(gray, current_corners, 500, 0.01, 7.0,
                          cv::noArray(), 3, true, 0.04);
  if (prev_frame_.empty() || prev_frame_.size() != gray.size()) {
    prev_frame_ = gray.clone();
    prev_corners_ = current_corners;
    return;
  }

  std::vector<cv::Point2f> tracked_corners;
  std::vector<uchar> status;
  std::vector<float> error;
  if (!prev_corners_.empty()) {
    cv::calcOpticalFlowPyrLK(prev_frame_, gray, prev_corners_,
                             tracked_corners, status, error);
  }

  std::vector<cv::Point2f> matched_prev, matched_curr;
  for (size_t index = 0; index < status.size(); ++index) {
    const cv::Point2f& point = tracked_corners[index];
    if (status[index] && point.x >= 0 && point.y >= 0 &&
        point.x < gray.cols && point.y < gray.rows) {
      matched_prev.push_back(prev_corners_[index]);
      matched_curr.push_back(point);
    }
  }

  std::vector<uchar> inlier_mask(matched_prev.size(), 0);
  if (matched_prev.size() >= 8) {
    inlierMaskComputation(matched_prev, matched_curr, &inlier_mask);
    inlier_mask.resize(matched_prev.size(), 0);
  }
  const size_t inliers = std::count_if(
      inlier_mask.begin(), inlier_mask.end(),
      [](uchar value) { return value != 0; });
  const float ratio = matched_prev.empty()
      ? 0.0f : static_cast<float>(inliers) / matched_prev.size();

  const float old_count = static_cast<float>(num_samples_);
  const float new_count = old_count + 1.0f;
  avg_num_keypoints_img1_ =
      (avg_num_keypoints_img1_ * old_count + prev_corners_.size()) / new_count;
  avg_num_keypoints_img2_ =
      (avg_num_keypoints_img2_ * old_count + current_corners.size()) / new_count;
  avg_num_matches_ =
      (avg_num_matches_ * old_count + matched_prev.size()) / new_count;
  avg_num_inliers_ =
      (avg_num_inliers_ * old_count + inliers) / new_count;
  avg_inlier_ratio_ =
      (avg_inlier_ratio_ * old_count + ratio) / new_count;
  ++num_samples_;

  show(frame, matched_prev, matched_curr);
  prev_frame_ = gray.clone();
  prev_corners_ = current_corners;
}

void LKFeatureTracker::show(const cv::Mat& frame,
                            std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  ros::NodeHandle private_nh("~");
  const bool visualize = private_nh.param<bool>("visualize", false);
  const bool save_outputs = private_nh.param<bool>("save_outputs", false);
  if (!visualize && !save_outputs) {
    return;
  }

  cv::Mat display;
  if (frame.channels() == 1) {
    cv::cvtColor(frame, display, cv::COLOR_GRAY2BGR);
  } else {
    display = frame.clone();
  }
  for (size_t index = 0; index < curr.size(); ++index) {
    cv::line(display, prev[index], curr[index], cv::Scalar(0, 255, 0), 1);
    cv::circle(display, curr[index], 2, cv::Scalar(0, 0, 255), -1);
  }

  if (save_outputs) {
    const std::string output_dir =
        private_nh.param<std::string>("output_dir", ".");
    const std::string path = output_dir + "/lk_tracks.png";
    try {
      if (!cv::imwrite(path, display)) {
        ROS_WARN_STREAM("Could not save " << path);
      }
    } catch (const cv::Exception& error) {
      ROS_WARN_STREAM("Could not save " << path << ": " << error.what());
    }
  }
  if (visualize) {
    cv::imshow(window_name_, display);
    cv::waitKey(1);
  }
}

void LKFeatureTracker::inlierMaskComputation(
    const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2,
    std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);
  CHECK_EQ(pts1.size(), pts2.size());
  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    cv::findFundamentalMat(pts1, pts2, CV_FM_RANSAC,
                           max_dist_from_epi_line_in_px, confidence_prob,
                           *inlier_mask);
  } catch (const cv::Exception& error) {
    ROS_WARN_STREAM("LK inlier mask could not be computed: " << error.what());
  }
}

void LKFeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  // LK has no descriptor ratio test; valid tracks are its good matches.
  std::cout << "Avg. Number of good matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ * 100.0f
            << "%" << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}
