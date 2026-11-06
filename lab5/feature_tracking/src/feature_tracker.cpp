#include "feature_tracker.h"

#include <algorithm>
#include <string>
#include <vector>
#include <numeric>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <ros/ros.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2021  - Lab 5 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a generic feature tracker base class
//  which you will augment with several derived classes for SIFT, SURF, ORB, and
//  FAST+BRIEF feature tracking.
//
// NOTE: Deliverables for the TEAM portion of this assignment start at number 3
// and end at number 7. If you have completed the parts labeled Deliverable 3-7,
// you are done with the TEAM portion of the lab. Deliverables 1-2 are
// individual.

using namespace cv;
using namespace cv::xfeatures2d;

/** Given two images, detect, describe and match features, then collect
 * statistics and optionally save visualizations.
 @param[in] img_1, img_2 Images where to track features.
 @param[out] matched_kp_1_kp_2 pair of vectors of keypoints with the same size
 so that matched_kp_1_kp_2.first[i] matches with matched_kp_1_kp_2.second[i].
*/
void FeatureTracker::trackFeatures(
    const cv::Mat& img_1, const cv::Mat& img_2,
    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>*
        matched_kp_1_kp_2) {
  if (img_1.empty() || img_2.empty()) {
    if (matched_kp_1_kp_2 != nullptr) {
      matched_kp_1_kp_2->first.clear();
      matched_kp_1_kp_2->second.clear();
    }
    ROS_WARN("Skipping an empty image pair.");
    return;
  }

  ros::NodeHandle private_nh("~");
  const int mode = private_nh.param<int>("mode", 0);
  const bool save_outputs = private_nh.param<bool>("save_outputs", mode == 0);
  const bool visualize = private_nh.param<bool>("visualize", false);
  const std::string output_dir =
      private_nh.param<std::string>("output_dir", ".");

  std::vector<KeyPoint> keypoints_1, keypoints_2;
  detectKeypoints(img_1, &keypoints_1);
  detectKeypoints(img_2, &keypoints_2);

  cv::Mat descriptors_1, descriptors_2;
  describeKeypoints(img_1, &keypoints_1, &descriptors_1);
  describeKeypoints(img_2, &keypoints_2, &descriptors_2);

  std::vector<std::vector<DMatch>> matches;
  std::vector<DMatch> good_matches;
  matchDescriptors(descriptors_1, descriptors_2, &matches, &good_matches);

  // The first neighbor of every query is the unfiltered match.
  std::vector<DMatch> raw_matches;
  raw_matches.reserve(matches.size());
  for (const auto& neighbors : matches) {
    if (!neighbors.empty()) {
      raw_matches.push_back(neighbors[0]);
    }
  }

  std::vector<KeyPoint> matched_1, matched_2;
  matched_1.reserve(good_matches.size());
  matched_2.reserve(good_matches.size());
  for (const auto& match : good_matches) {
    matched_1.push_back(keypoints_1.at(match.queryIdx));
    matched_2.push_back(keypoints_2.at(match.trainIdx));
  }
  if (matched_kp_1_kp_2 != nullptr) {
    matched_kp_1_kp_2->first = matched_1;
    matched_kp_1_kp_2->second = matched_2;
  }

  std::vector<uchar> inlier_mask(good_matches.size(), 0);
  if (good_matches.size() >= 8) {
    inlierMaskComputation(matched_1, matched_2, &inlier_mask);
    // OpenCV can return an empty mask when the geometry is degenerate.
    inlier_mask.resize(good_matches.size(), 0);
  }
  const unsigned int num_inliers =
      static_cast<unsigned int>(std::count_if(
          inlier_mask.begin(), inlier_mask.end(),
          [](uchar value) { return value != 0; }));
  const float inlier_ratio = good_matches.empty()
      ? 0.0f
      : static_cast<float>(num_inliers) / good_matches.size();

  if (save_outputs || visualize) {
    cv::Mat keypoints_img_1, keypoints_img_2;
    cv::drawKeypoints(img_1, keypoints_1, keypoints_img_1);
    cv::drawKeypoints(img_2, keypoints_2, keypoints_img_2);

    cv::Mat raw_img, good_img, inliers_img;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2,
                    raw_matches, raw_img, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2,
                    good_matches, good_img, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2,
                    good_matches, inliers_img, cv::Scalar(0, 0, 255),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    const std::vector<char> char_mask(inlier_mask.begin(),
                                      inlier_mask.end());
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2,
                    good_matches, inliers_img, cv::Scalar(0, 255, 0),
                    cv::Scalar::all(-1), char_mask,
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS |
                        cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

    if (save_outputs) {
      const auto save_image = [&output_dir](const std::string& name,
                                            const cv::Mat& image) {
        const std::string path = output_dir + "/" + name;
        try {
          if (cv::imwrite(path, image)) {
            ROS_INFO_STREAM("Saved " << path);
          } else {
            ROS_WARN_STREAM("Could not save " << path);
          }
        } catch (const cv::Exception& error) {
          ROS_WARN_STREAM("Could not save " << path << ": " << error.what());
        }
      };
      save_image("keypoints_1.png", keypoints_img_1);
      save_image("keypoints_2.png", keypoints_img_2);
      save_image("matches_raw.png", raw_img);
      save_image("matches_ratio.png", good_img);
      save_image("matches_inliers.png", inliers_img);
    }
    if (visualize) {
      cv::imshow("Keypoints 1", keypoints_img_1);
      cv::imshow("Keypoints 2", keypoints_img_2);
      cv::imshow("Matches and inliers", inliers_img);
      cv::waitKey(mode == 0 ? 0 : 1);
    }
  }

  const float new_num_samples = static_cast<float>(num_samples_) + 1.0f;
  const float old_num_samples = static_cast<float>(num_samples_);
  avg_num_keypoints_img1_ =
      (avg_num_keypoints_img1_ * old_num_samples + keypoints_1.size()) /
      new_num_samples;
  avg_num_keypoints_img2_ =
      (avg_num_keypoints_img2_ * old_num_samples + keypoints_2.size()) /
      new_num_samples;
  avg_num_matches_ =
      (avg_num_matches_ * old_num_samples + raw_matches.size()) /
      new_num_samples;
  avg_num_good_matches_ =
      (avg_num_good_matches_ * old_num_samples + good_matches.size()) /
      new_num_samples;
  avg_num_inliers_ =
      (avg_num_inliers_ * old_num_samples + num_inliers) / new_num_samples;
  avg_inlier_ratio_ =
      (avg_inlier_ratio_ * old_num_samples + inlier_ratio) /
      new_num_samples;
  ++num_samples_;

  if (mode == 0) {
    ROS_INFO_STREAM("Descriptor dimension: " << descriptors_1.cols
                    << "; keypoints: " << keypoints_1.size() << ", "
                    << keypoints_2.size() << "; raw matches: "
                    << raw_matches.size() << "; good matches: "
                    << good_matches.size() << "; inliers: " << num_inliers
                    << "; inlier ratio: " << inlier_ratio);
  }
}

/** Compute Inlier Mask out of the given matched keypoints.
   *  Both keypoints_1 and keypoints_2 input parameters must be ordered by match
   * i.e. keypoints_1[0] has been matched to keypoints_2[0].
   * Therefore, both keypoints vectors must have the same length.
    @param[in] keypoints_1 List of keypoints detected on the first image.
    @param[in] keypoints_2 List of keypoints detected on the second image.
    @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
  */
void FeatureTracker::inlierMaskComputation(const std::vector<KeyPoint> &keypoints_1,
                                           const std::vector<KeyPoint> &keypoints_2,
                                           std::vector<uchar> *inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);
  const size_t size = keypoints_1.size();
  CHECK_EQ(keypoints_2.size(), size) << "Size of keypoint vectors "
                                        "should be the same!";

  std::vector<Point2f> pts1(size);
  std::vector<Point2f> pts2(size);
  for (size_t i = 0; i < keypoints_1.size(); i++) {
    pts1[i] = keypoints_1[i].pt;
    pts2[i] = keypoints_2[i].pt;
  }

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, CV_FM_RANSAC, max_dist_from_epi_line_in_px, confidence_prob, *inlier_mask);
  } catch (...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}

/** Example of function to draw matches. Feel free to re-use this example or
 *  create your own. You will need to modify it in order to plot the different
 *  figures. You can add more functions to this class if needed.
 */
void FeatureTracker::drawMatches(const cv::Mat &img_1,
                                 const cv::Mat &img_2,
                                 const std::vector<KeyPoint> &keypoints_1,
                                 const std::vector<KeyPoint> &keypoints_2,
                                 const std::vector<std::vector<DMatch>> &matches) {
  cv::namedWindow("tracked_features", cv::WINDOW_NORMAL);
  cv::Mat img_matches;
  cv::drawMatches(img_1,
                  keypoints_1,
                  img_2,
                  keypoints_2,
                  matches,
                  img_matches,
                  Scalar::all(-1),
                  Scalar::all(-1),
                  std::vector<std::vector<char>>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  // Show detected matches
  imshow("tracked_features", img_matches);

  // Wait indefinitely for key pressed, but allow ROS to also kill everything.
  // Otherwise ROS will not die unless we close the window.
  // Use this when wanting to visualize pairs of images, one at a time.
  while (ros::ok() && waitKey(10) == -1) {} // Instead of using waitKey(0);

  // Alternatively, just wait for some seconds. Use this when playing with video
  // sequences.
  // waitKey(10);
}

void FeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of good matches: " << avg_num_good_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ * 100.0f
            << "%" << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}
