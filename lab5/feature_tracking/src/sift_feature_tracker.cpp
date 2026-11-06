#include "sift_feature_tracker.h"

#include <vector>
#include <glog/logging.h>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

/**
   Sift feature tracker Constructor.
*/
SiftFeatureTracker::SiftFeatureTracker()
  : FeatureTracker(),
    detector(SIFT::create()) {}

/** Detect keypoints in an image.
    @param[in] img Image input where to detect keypoints.
    @param[out] keypoints List of keypoints detected on the given image.
*/
void SiftFeatureTracker::detectKeypoints(const cv::Mat& img,
                                         std::vector<KeyPoint>* keypoints) const {
  CHECK_NOTNULL(keypoints);
  detector->detect(img, *keypoints);
}

/** Describe keypoints in an image.
    @param[in] img Image used to detect the keypoints.
    @param[in, out] keypoints List of keypoints detected on the image. Depending
    on the detector used, some keypoints might be added or removed.
    @param[out] descriptors List of descriptors for the given keypoints.
*/
void SiftFeatureTracker::describeKeypoints(const cv::Mat& img,
                                           std::vector<KeyPoint>* keypoints,
                                           cv::Mat* descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);
  detector->compute(img, *keypoints, *descriptors);
}

/** Match descriptors and apply the distance-ratio test.
    @param[in] descriptors_1 First list of descriptors.
    @param[in] descriptors_2 Second list of descriptors.
    @param[out] matches List of k best matches between descriptors.
    @param[out] good_matches List of descriptors classified as "good"
*/
void SiftFeatureTracker::matchDescriptors(
                                          const cv::Mat& descriptors_1,
                                          const cv::Mat& descriptors_2,
                                          std::vector<std::vector<DMatch>>* matches,
                                          std::vector<cv::DMatch>* good_matches) const {
  CHECK_NOTNULL(matches);
  CHECK_NOTNULL(good_matches);
  matches->clear();
  good_matches->clear();

  // FLANN needs two training descriptors for the two-nearest-neighbor test.
  if (descriptors_1.empty() || descriptors_2.rows < 2) {
    return;
  }

  FlannBasedMatcher flann_matcher;

  flann_matcher.knnMatch(descriptors_1, descriptors_2, *matches, 2);
  for (const auto& neighbors : *matches) {
    if (neighbors.size() >= 2 &&
        neighbors[0].distance < 0.8f * neighbors[1].distance) {
      good_matches->push_back(neighbors[0]);
    }
  }
}
