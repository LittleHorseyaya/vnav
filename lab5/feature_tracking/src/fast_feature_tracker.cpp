// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2019  - Lab 5 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement an ORB feature tracker derived class
//  that inherits from your FeatureTracker base class.
//
// NOTE: Deliverables for the TEAM portion of this assignment start at number 3
// and end at number 7. If you have completed the parts labeled Deliverable 3-7,
// you are done with the TEAM portion of the lab. Deliverables 1-2 are
// individual.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  DELIVERABLE 6 (continued) | Comparing Feature Matching on Real Data
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
// For this part, you will need to implement the same functions you've just
// implemented in the case of SIFT, SURF, and ORB, but now for FAST+BRIEF.
//
// NOTE: We do not provide much template code for this section or the corresponding
// header file (fast_feature_tracker.h), but you should define and implement
// the same set of functions as for SIFT and SURF (see the template code for
// those cases for reference as needed)
//
// NOTE: Don't forget to modify the CMakeLists.txt to add these files!
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
#include "fast_feature_tracker.h"
#include <algorithm>
#include <glog/logging.h>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

FastFeatureTracker::FastFeatureTracker()
    : detector(FastFeatureDetector::create(20, true)),
      descriptor(BriefDescriptorExtractor::create(32)) {}

void FastFeatureTracker::detectKeypoints(const cv::Mat &img,
                                         std::vector<cv::KeyPoint> *keypoints) const {
  CHECK_NOTNULL(keypoints);
  detector->detect(img, *keypoints);
  // Keep the strongest 500 FAST corners, matching ORB's feature budget.
  if (keypoints->size() > 500) {
    std::partial_sort(keypoints->begin(), keypoints->begin() + 500,
                      keypoints->end(),
                      [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                        return a.response > b.response;
                      });
    keypoints->resize(500);
  }
}

void FastFeatureTracker::describeKeypoints(const cv::Mat &img,
                                           std::vector<cv::KeyPoint> *keypoints,
                                           cv::Mat *descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);
  if (keypoints->empty()) {
    descriptors->release();
    return;
  }
  // BRIEF drops corners too close to the image edge.
  descriptor->compute(img, *keypoints, *descriptors);
}

void FastFeatureTracker::matchDescriptors(const cv::Mat &descriptors_1,
                                          const cv::Mat &descriptors_2,
                                          std::vector<std::vector<cv::DMatch>> *matches,
                                          std::vector<cv::DMatch> *good_matches) const {
  CHECK_NOTNULL(matches);
  CHECK_NOTNULL(good_matches);
  matches->clear();
  good_matches->clear();
  if (descriptors_1.empty() || descriptors_2.rows < 2) {
    return;
  }

  // BRIEF produces binary descriptors; Hamming distance preserves their metric.
  BFMatcher matcher(NORM_HAMMING);
  matcher.knnMatch(descriptors_1, descriptors_2, *matches, 2);
  for (const auto& neighbors : *matches) {
    if (neighbors.size() >= 2 &&
        neighbors[0].distance < 0.8f * neighbors[1].distance) {
      good_matches->push_back(neighbors[0]);
    }
  }
}
