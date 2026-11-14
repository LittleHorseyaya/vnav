/*
 * @file pose_estimation.cpp
 * @brief Estimates the pose from frame to frame.
 */


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <memory>
#include <cmath>
#include <iomanip>
#include <deque>
#include <limits>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Eigen>
#include <Eigen/SVD>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// OpenGV
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/point_cloud/methods.hpp>

#include "lab6_utils.h"
#include "pose_estimation.h"
#include "tracker_shim.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2021  - Lab 6 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to use the following methods for motion estimation:
//  - Nister's 5 point algorithm (2D-2D correspondences)
//  - Longuet-Higgins 8 point algorithm (2D-2D correspondences)
//  - OpenGV's 2 point algorithm (2D-2D correspondences, known rotation)
//  - Arun's 3 point algorithm (3D-3D correspondences)
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// These are Google Flags (also known as gflags).
// Have a look at the provided roslaunch to see how to modify them on the fly.
// Feel free to use ROS parameters, or command-line flags if you prefer.
DEFINE_bool(use_ransac, true, "Use Random Sample Consensus.");
DEFINE_string(rpe_csv, "", "Path to the RPE CSV output.");
DEFINE_string(match_cache, "", "New cache prefix; writes PREFIX_frames.csv and PREFIX_matches.csv.");
DEFINE_bool(cache_only, false, "Save matched inputs and aligned GT, skipping pose solvers.");
DEFINE_bool(fixed_seed, false, "Use fixed OpenGV and OpenCV seeds for controlled tests.");
DEFINE_double(twopt_threshold, 1e-5, "Two-point RANSAC bearing residual threshold.");
DEFINE_bool(scale_translation, true, "Whether to scale estimated translation to match ground truth scale");
DEFINE_int32(pose_estimator, 0,
             "Pose estimation algorithm, valid values are:"
             "0 for OpengGV's 5-point algorithm."
             "1 for OpengGV's 8-point algorithm."
             "2 for OpengGV's 2-point algorithm."
             "3 for Arun's 3-point method.");

using namespace std;
namespace enc = sensor_msgs::image_encodings;

// Global definitions.
// Mono OpenGV RANSAC problems and adapters.
// See OpenGV examples on how to use:
// https://laurentkneip.github.io/opengv/page_how_to_use.html
using RansacProblem = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using Adapter = opengv::relative_pose::CentralRelativeAdapter;
using RansacProblemGivenRot = opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
using AdapterGivenRot = opengv::relative_pose::CentralRelativeAdapter;
using Adapter3D = opengv::point_cloud::PointCloudAdapter;
using RansacProblem3D = opengv::sac_problems::point_cloud::PointCloudSacProblem;

// Global variables. As a general coding style, global variables are followed
// by a trailing underscore (Google coding style).
std::unique_ptr<TrackerWrapper> feature_tracker_;
ros::Publisher pub_pose_estimation_, pub_pose_gt_;
ros::Subscriber sub_cinfo_;
geometry_msgs::PoseStamped curr_pose_;
geometry_msgs::PoseStamped prev_pose_;

// See definition of CameraParams in lab6_utils.h
// We declare these global variables, and we assign values to them later in the main function
CameraParams camera_params_;
cv::Mat R_camera_body, t_camera_body;
cv::Mat T_camera_body;
geometry_msgs::Pose pose_camera_body;
tf::Transform transform_camera_body;

// All callbacks run on the existing single-threaded spinOnce loop.
// Buffer BODY poses, interpolate in time, then apply the camera extrinsic.
std::deque<geometry_msgs::PoseStamped> gt_body_buffer_;
using ImagePair = std::pair<sensor_msgs::ImageConstPtr,
                            sensor_msgs::ImageConstPtr>;
std::deque<ImagePair> pending_image_pairs_;
ros::Time previous_image_stamp_;
double current_gt_bracket_s_ = 0.0;

void processCameraPair(const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg);

bool interpolateCameraPose(const ros::Time& stamp,
                           geometry_msgs::PoseStamped& output) {
  if (gt_body_buffer_.empty() || stamp < gt_body_buffer_.front().header.stamp ||
      stamp > gt_body_buffer_.back().header.stamp) {
    return false;  // Never extrapolate.
  }

  size_t hi = 0;
  while (hi < gt_body_buffer_.size() &&
         gt_body_buffer_[hi].header.stamp < stamp) {
    ++hi;
  }
  if (hi == gt_body_buffer_.size()) return false;

  tf::Transform body_pose;
  if (gt_body_buffer_[hi].header.stamp == stamp) {
    tf::poseMsgToTF(gt_body_buffer_[hi].pose, body_pose);
    current_gt_bracket_s_ = 0.0;
  } else {
    if (hi == 0) return false;
    const auto& a = gt_body_buffer_[hi - 1];
    const auto& b = gt_body_buffer_[hi];
    const double dt = (b.header.stamp - a.header.stamp).toSec();
    if (dt <= 0.0 || dt > 0.05) return false;
    const double alpha = (stamp - a.header.stamp).toSec() / dt;
    tf::Transform ta, tb;
    tf::poseMsgToTF(a.pose, ta);
    tf::poseMsgToTF(b.pose, tb);
    const tf::Vector3 position =
        ta.getOrigin() * (1.0 - alpha) + tb.getOrigin() * alpha;
    tf::Quaternion qa = ta.getRotation().normalized();
    tf::Quaternion qb = tb.getRotation().normalized();
    tf::Quaternion q = qa.slerp(qb, alpha);
    q.normalize();
    body_pose = tf::Transform(q, position);
    current_gt_bracket_s_ = dt;
  }

  output.header.stamp = stamp;
  output.header.frame_id = "world";
  tf::poseTFToMsg(body_pose * transform_camera_body, output.pose);
  return true;
}

void processPendingImages() {
  while (!pending_image_pairs_.empty() && !gt_body_buffer_.empty()) {
    const ImagePair pair = pending_image_pairs_.front();
    const ros::Time stamp = pair.first->header.stamp;
    if (stamp > gt_body_buffer_.back().header.stamp) {
      return;  // Wait for a FUTURE odometry sample to bracket the image.
    }
    pending_image_pairs_.pop_front();
    geometry_msgs::PoseStamped aligned;
    if (!interpolateCameraPose(stamp, aligned)) {
      ROS_WARN_STREAM("GT_SYNC skipped image=" << std::setprecision(17)
                      << stamp.toSec() << ": no valid GT bracket (max 0.05 s).");
      continue;
    }
    curr_pose_ = aligned;
    processCameraPair(pair.first, pair.second);
  }
}

void cameraCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                    const sensor_msgs::ImageConstPtr& depth_msg) {
  const double rgb_depth_dt =
      std::abs((rgb_msg->header.stamp - depth_msg->header.stamp).toSec());
  if (rgb_depth_dt > 0.001) {
    ROS_WARN_STREAM("GT_SYNC skipped RGB/depth pair, dt=" << rgb_depth_dt);
    return;
  }
  if (pending_image_pairs_.size() >= 30) {
    ROS_WARN("GT_SYNC pending image queue overflow; dropping oldest pair.");
    pending_image_pairs_.pop_front();
  }
  pending_image_pairs_.emplace_back(rgb_msg, depth_msg);
  processPendingImages();
}


void poseCallbackTesse(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::PoseStamped body;
  body.header = msg->header;
  body.pose = msg->pose.pose;
  const auto& p = body.pose.position;
  const auto& q = body.pose.orientation;
  const double qnorm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(qnorm2) || qnorm2 < 1e-12) {
    ROS_WARN("GT_SYNC invalid odometry pose; ignoring sample.");
    return;
  }
  if (!gt_body_buffer_.empty() &&
      body.header.stamp < gt_body_buffer_.back().header.stamp) {
    ROS_ERROR("GT_SYNC out-of-order GT/time reset. Restart node before replay.");
    return;
  }
  if (!gt_body_buffer_.empty() &&
      body.header.stamp == gt_body_buffer_.back().header.stamp) {
    gt_body_buffer_.back() = body;
  } else {
    gt_body_buffer_.push_back(body);
  }

  // Publish the original GT stream without changing the image-aligned snapshot.
  tf::Transform body_tf;
  tf::poseMsgToTF(body.pose, body_tf);
  tf::Quaternion normalized_q = body_tf.getRotation();
  normalized_q.normalize();
  body_tf.setRotation(normalized_q);
  geometry_msgs::PoseStamped camera_gt;
  camera_gt.header = body.header;
  camera_gt.header.frame_id = "world";
  tf::poseTFToMsg(body_tf * transform_camera_body, camera_gt.pose);
  pub_pose_gt_.publish(camera_gt);

  processPendingImages();
  // Retain at most ~10 s at 200 Hz; never grow memory without bound.
  while (gt_body_buffer_.size() > 2000) gt_body_buffer_.pop_front();
}

/**
 * @brief      Compute 3D bearing vectors from pixel points
 *
 * @param[in]  pts1              Feature correspondences from camera 1
 * @param[in]  pts2              Feature correspondences from camera 2
 * @param      bearing_vector_1  Bearing vector to pts1 in camera 1
 * @param      bearing_vector_2  Bearing vector to pts2 in camera 2
 */
void calibrateKeypoints(const std::vector<cv::Point2f>& pts1,
                        const std::vector<cv::Point2f>& pts2,
                        opengv::bearingVectors_t& bearing_vector_1,
                        opengv::bearingVectors_t& bearing_vector_2) {
    //
    // For this part, we perform:
    //   1. Use the function cv::undistortPoints to rectify the keypoints.
    //   2. Return the bearing vectors for each keypoint.
    //

    std::vector<cv::Point2f> points1_rect, points2_rect;
    cv::undistortPoints(pts1, points1_rect, camera_params_.K, camera_params_.D);
    cv::undistortPoints(pts2, points2_rect, camera_params_.K, camera_params_.D);

    for (auto const& pt: points1_rect){
      opengv::bearingVector_t bearing_vector(pt.x, pt.y, 1); // focal length is 1 after undistortion
      bearing_vector_1.push_back(bearing_vector.normalized());
    }

    for (auto const& pt: points2_rect){
      opengv::bearingVector_t bearing_vector(pt.x, pt.y, 1); // focal length is 1 after undistortion
      bearing_vector_2.push_back(bearing_vector.normalized());
    }
}

/**
 * @brief      Update pose estimate using previous absolue pose and estimated relative pose
 *
 * @param[in]  prev_pose         ground-truth absolute pose of previous frame
 * @param[in]  relative_pose     estimated relative pose between current frame and previous frame
 * @param      output            estimated absolute pose of current frame
 */
void updatePoseEstimate(geometry_msgs::Pose const& prev_pose, geometry_msgs::Pose const& relative_pose, geometry_msgs::Pose& output) {
  tf::Transform prev, relative;
  tf::poseMsgToTF(prev_pose, prev);
  tf::poseMsgToTF(relative_pose, relative);
  tf::poseTFToMsg(prev*relative, output);
}

/**
 * @brief      Given an estimated translation up to scale, return an absolute translation with correct scale using ground truth
 *
 * @param[in]  prev_pose         ground-truth absolute pose of previous frame
 * @param[in]  curr_pose         ground-truth absolute pose of current frame
 * @param      translation       estimated translation between current frame and previous frame
 */
void scaleTranslation(geometry_msgs::Point& translation, geometry_msgs::PoseStamped const& prev_pose, geometry_msgs::PoseStamped const& curr_pose) {
  if (!FLAGS_scale_translation) return;
  tf::Transform prev, curr;
  tf::poseMsgToTF(prev_pose.pose, prev);
  tf::poseMsgToTF(curr_pose.pose, curr);
  tf::Transform const relative_pose = prev.inverseTimes(curr);
  double const translation_scale = relative_pose.getOrigin().length();
  if (isnan(translation_scale) || isinf(translation_scale)) {
    ROS_WARN("Failed to scale translation");
    return;
  }
  double const old_scale = sqrt(pow(translation.x, 2) + pow(translation.y, 2) + pow(translation.z, 2));
  if (!std::isfinite(old_scale) || old_scale < 1e-8) {
    ROS_WARN("Cannot recover scale: invalid or near-zero translation.");
    return;
  }
  translation.x *= translation_scale / old_scale;
  translation.y *= translation_scale / old_scale;
  translation.z *= translation_scale / old_scale;
}


/** @brief     (TODO) Compute Relative Pose Error (RPE) in translation and rotation.
    @param[in] gt_t_prev_frame ground-truth transform for previous frame.
    @param[in] gt_t_curr_frame ground-truth transform for current frame.
    @param[in] est_t_prev_frame estimated transform for previous frame.
    @param[in] est_t_curr_frame estimated transform for current frame.
*/
void evaluateRPE(const tf::Transform& gt_t_prev_frame,
                 const tf::Transform& gt_t_curr_frame,
                 const tf::Transform& est_t_prev_frame,
                 const tf::Transform& est_t_curr_frame,
                 const ros::Time& image_stamp) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 4 (Part 2) | Evaluate Relative Pose Error (RPE)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //   1. Follow the equations given in the README to compute the relative pose
  //     error at each time frame
  //   2. Print the error in translation and rotation.
  //   3. For plotting the statics, you might want to log the errors in a .txt or .csv file
  //
  // Hint: You might find tf::vectorTFToEigen and tf::matrixTFToEigen helpful

  tf::Transform const est_relative_pose = est_t_prev_frame.inverseTimes(est_t_curr_frame);
  tf::Transform const gt_relative_pose = gt_t_prev_frame.inverseTimes(gt_t_curr_frame);

  // ***************************** begin solution *****************************

  Eigen::Matrix3d R_gt, R_est;
  tf::matrixTFToEigen(gt_relative_pose.getBasis(), R_gt);
  tf::matrixTFToEigen(est_relative_pose.getBasis(), R_est);

  const double rotation_error = (R_gt - R_est).norm();

  Eigen::Vector3d t_gt, t_est;
  tf::vectorTFToEigen(gt_relative_pose.getOrigin(), t_gt);
  tf::vectorTFToEigen(est_relative_pose.getOrigin(), t_est);

  if (!R_gt.allFinite() || !R_est.allFinite() ||
      !t_gt.allFinite() || !t_est.allFinite()) {
    ROS_WARN("Skipping non-finite RPE.");
    return;
  }

  if (FLAGS_pose_estimator < 3) {
    if (t_gt.norm() < 1e-8 || t_est.norm() < 1e-8) {
      ROS_WARN("Skipping direction RPE: near-zero translation.");
      return;
    }
    t_gt.normalize();
    t_est.normalize();
  }

  const double translation_error = (t_gt - t_est).norm();
  if (!FLAGS_rpe_csv.empty()) {
    static std::ofstream csv(FLAGS_rpe_csv);
    static bool header_written = false;

    if (!csv.is_open()) {
      ROS_ERROR_STREAM("Cannot open CSV: " << FLAGS_rpe_csv);
    } else {
      if (!header_written) {
        csv << "image_time_s,method,rotation_frobenius,"
               "translation_error,translation_unit,previous_image_time_s,frame_dt_s,gt_bracket_s\n";
        header_written = true;
      }
      csv << std::setprecision(17)
          << image_stamp.toSec() << ","
          << FLAGS_pose_estimator << ","
          << rotation_error << ","
          << translation_error << ","
          << (FLAGS_pose_estimator < 3 ? "direction" : "m") << ","
          << previous_image_stamp_.toSec() << ","
          << (image_stamp - previous_image_stamp_).toSec() << ","
          << current_gt_bracket_s_
          << std::endl;
    }
  }

  ROS_INFO_STREAM("RPE rotation (Frobenius): " << rotation_error
                  << ", translation: " << translation_error
                  << (FLAGS_pose_estimator < 3 ? " (direction)" : " m"));
  // ***************************** end solution *****************************

}

/** @brief (TODO) This function is called when a new image is published. This is
 *   where all the magic happens for this lab
 *  @param[in]  rgb_msg    RGB Camera message
 *  @param[in]  depth_msg  Depth Camera message
 */
void processCameraPair(const sensor_msgs::ImageConstPtr &rgb_msg, const sensor_msgs::ImageConstPtr &depth_msg) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 4-5 | A Suite of Methods for Motion Estimation
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //   1. (Deliverable 4) 2D-2D Correspondences
  //     - 5 point Nister (with RANSAC)
  //     - 8 point Longuet-Higgins (with RANSAC)
  //     - 2 point OpenGV (with RANSAC)
  //   2. (Deliverable 5) 3D-3D Correspondences
  //     - 3 point Arun (with and without RANSAC)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  cv::Mat bgr, depth;

  try {
    // Convert ROS msg type to OpenCV image type.
    bgr = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
    depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert rgb or depth images.");
    return;
  }

  // for drawing
  cv::Mat view = bgr.clone();

  // cv::imshow("view", view);

  // The static keyword makes this variable to be created only once.
  // Therefore, this line of code will only be evaluated the first time
  // cameraCallback is called, but never again.
  // Nevertheless, the variable `prev_image` will continue to be available.
  // store images from previous time step
  static cv::Mat prev_bgr;
  static cv::Mat prev_depth;
  auto save_current_frame = [&]() {
    prev_bgr = bgr.clone();
    prev_depth = depth.clone();
    prev_pose_ = curr_pose_;
    previous_image_stamp_ = rgb_msg->header.stamp;
  };
  // Wait until a ground-truth pose has arrived.
  if (curr_pose_.header.frame_id.empty()) {
    return;
  }

  // The first valid image initializes the previous frame.
  if (prev_bgr.empty()) {
    prev_bgr = bgr.clone();
    prev_depth = depth.clone();
    prev_pose_ = curr_pose_;
    previous_image_stamp_ = rgb_msg->header.stamp;
    return;
  }
  ROS_INFO_STREAM("Time check: image=" << rgb_msg->header.stamp.toSec()
                  << ", gt=" << curr_pose_.header.stamp.toSec()
                  << ", lag="
                  << (rgb_msg->header.stamp -
                      curr_pose_.header.stamp).toSec());
  // Track features returns the 2D-2D matches between images
  // in pixels (pixel coords in image 1 -> pixel coords in image 2).
  std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> matched_kp_1_kp_2;
  if (FLAGS_fixed_seed) cv::setRNGSeed(12345);
  feature_tracker_->track(prev_bgr, bgr, &matched_kp_1_kp_2);

  int N = matched_kp_1_kp_2.first.size();
  std::cout << "Matched " << N << " keypoints" << std::endl;
  std::vector<cv::Point2f> pts1, pts2;

  cv::KeyPoint::convert(matched_kp_1_kp_2.first, pts1);
  cv::KeyPoint::convert(matched_kp_1_kp_2.second, pts2);

  // (TODO) undistort the image keypoints and convert them to bearing vectors
  // using camera intrinsics
  // Use the calibrateKeypoints function defined
  // Store the results in bearing_vector_1 and bearing_vector_2 defined here.
  opengv::bearingVectors_t bearing_vector_1, bearing_vector_2;
  // ***************************** begin solution *****************************
  if (!pts1.empty() && pts1.size() == pts2.size()) {
    calibrateKeypoints(pts1, pts2, bearing_vector_1, bearing_vector_2);
  }


  // ***************************** end solution *****************************

  // Cache common inputs BEFORE any solver changes bearing vectors or scales translation.
  // frames.csv commits each complete frame; matches.csv contains its correspondences.
  if (!FLAGS_match_cache.empty()) {
    static std::ofstream cache_frames;
    static std::ofstream cache_matches;
    static bool cache_opened = false;
    static size_t cache_frame_id = 0;
    if (!cache_opened) {
      const std::string frames_path = FLAGS_match_cache + "_frames.csv";
      const std::string matches_path = FLAGS_match_cache + "_matches.csv";
      std::ifstream existing_frames(frames_path), existing_matches(matches_path);
      if (existing_frames.good() || existing_matches.good()) {
        ROS_FATAL("CACHE refuses to overwrite existing files. Choose a new prefix.");
        ros::shutdown();
        return;
      }
      cache_frames.open(frames_path);
      cache_matches.open(matches_path);
      if (!cache_frames.is_open() || !cache_matches.is_open()) {
        ROS_FATAL("CACHE cannot open output files; check directory and permissions.");
        ros::shutdown();
        return;
      }
      cache_frames << "frame_id,previous_image_time_s,image_time_s,match_count,"
                      "gt_bracket_s,r00,r01,r02,r10,r11,r12,r20,r21,r22,tx,ty,tz\n";
      cache_matches << "frame_id,match_id,b1x,b1y,b1z,b2x,b2y,b2z,depth1_m,depth2_m\n";
      cache_frames << std::setprecision(17);
      cache_matches << std::setprecision(17);
      cache_opened = true;
    }

    const size_t count = bearing_vector_1.size();
    if (count != bearing_vector_2.size() || count != pts1.size() || count != pts2.size()) {
      ROS_ERROR("CACHE inconsistent match array sizes; skipping pair.");
      save_current_frame();
      return;
    }
    auto sample_depth = [](const cv::Mat& image, const cv::Point2f& point) {
      const double invalid = std::numeric_limits<double>::quiet_NaN();
      if (image.type() != CV_32FC1 ||
          !std::isfinite(point.x) || !std::isfinite(point.y)) return invalid;
      const int x = static_cast<int>(std::floor(point.x));
      const int y = static_cast<int>(std::floor(point.y));
      if (x < 0 || x >= image.cols || y < 0 || y >= image.rows) return invalid;
      const double d = image.at<float>(y, x);
      return (std::isfinite(d) && d > 0.0) ? d : invalid;
    };

    for (size_t i = 0; i < count; ++i) {
      cache_matches << cache_frame_id << "," << i;
      for (int j = 0; j < 3; ++j) cache_matches << "," << bearing_vector_1[i](j);
      for (int j = 0; j < 3; ++j) cache_matches << "," << bearing_vector_2[i](j);
      cache_matches << "," << sample_depth(prev_depth, pts1[i])
                    << "," << sample_depth(depth, pts2[i]) << "\n";
    }
    cache_matches.flush();
    if (!cache_matches.good()) {
      ROS_FATAL("CACHE match write failed; stopping without committing this frame.");
      ros::shutdown();
      return;
    }
    tf::Transform prev_gt, curr_gt;
    tf::poseMsgToTF(prev_pose_.pose, prev_gt);
    tf::poseMsgToTF(curr_pose_.pose, curr_gt);
    const tf::Transform relative_gt = prev_gt.inverseTimes(curr_gt);
    cache_frames << cache_frame_id << "," << previous_image_stamp_.toSec()
                 << "," << rgb_msg->header.stamp.toSec() << "," << count
                 << "," << current_gt_bracket_s_;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        cache_frames << "," << relative_gt.getBasis()[row][col];
    for (int j = 0; j < 3; ++j) cache_frames << "," << relative_gt.getOrigin()[j];
    cache_frames << std::endl;
    if (!cache_frames.good()) {
      ROS_FATAL("CACHE frame write failed; stopping.");
      ros::shutdown();
      return;
    }
    ROS_INFO_STREAM("CACHE saved frame=" << cache_frame_id << " matches=" << count
                    << " dt=" << (rgb_msg->header.stamp - previous_image_stamp_).toSec());
    ++cache_frame_id;
    if (FLAGS_cache_only) {
      save_current_frame();
      return;
    }
  }

  // We create the central relative adapter, have a look at OpenGV's
  // documentation to understand how this class works.
  Adapter adapter_mono (bearing_vector_1, bearing_vector_2);

  // (TODO) Compute the pose estimate using the different techniques
  // here. You should fill the variable `pose_estimate` with your pose
  // estimation.
  geometry_msgs::PoseStamped pose_estimation;
  tf::poseTFToMsg(tf::Pose::getIdentity(), pose_estimation.pose);

  // initialize a relative pose estimate, you will update relative_pose_estimate using the following algorithms
  geometry_msgs::Pose relative_pose_estimate = pose_estimation.pose;

  switch(FLAGS_pose_estimator) {
  case 0: {
    // *********************************************************************************
    // ******************************** 5-point Method *********************************
    // *********************************************************************************

    // First, ensure we have the minimal number of correspondences to compute the relative pose.
    static constexpr size_t min_nr_of_correspondences = 5;
    if (adapter_mono.getNumberCorrespondences() >= min_nr_of_correspondences) {
      if (!FLAGS_use_ransac) {
        // Without RANSAC [OPTIONAL]
        // Consider using the "extractPose" function provided to obtain inliers, rotation, and translation

        // ***************************** begin solution *****************************


        // ***************************** end solution *****************************
      } else {
        // (TODO) With RANSAC
        // Consider using ``eigen2Pose`` to convert the solution of ransac to ``relative_pose_estimate``

        // ***************************** begin solution *****************************

        opengv::sac::Ransac<RansacProblem> ransac;
        auto problem = std::make_shared<RansacProblem>(
            adapter_mono, RansacProblem::NISTER, !FLAGS_fixed_seed);

        if (adapter_mono.getNumberCorrespondences() <
            static_cast<size_t>(problem->getSampleSize())) {
          ROS_WARN("Too few matches for NISTER RANSAC.");
          save_current_frame();
          return;
        }

        ransac.sac_model_ = problem;
        ransac.threshold_ = 1e-6;
        ransac.max_iterations_ = 1000;

        if (!ransac.computeModel() || !ransac.model_coefficients_.allFinite()) {
          ROS_WARN("NISTER RANSAC failed.");
          save_current_frame();
          return;
        }

        opengv::transformation_t refined_model = ransac.model_coefficients_;

        if (ransac.inliers_.size() >= 8) {
          problem->optimizeModelCoefficients(
              ransac.inliers_, ransac.model_coefficients_, refined_model);
        }

        if (!refined_model.allFinite()) {
          ROS_WARN("NISTER refinement failed.");
          save_current_frame();
          return;
        }

        relative_pose_estimate = eigen2Pose(refined_model);

        ROS_INFO_STREAM("NISTER inliers: " << ransac.inliers_.size()
                        << "/" << bearing_vector_1.size());

        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to compute pose estimation using"
               " Nister's algorithm.");
      save_current_frame();
      return;
    }
    break;
  }
  case 1: {
    // *********************************************************************************
    // ******************************** 8-point Method *********************************
    // *********************************************************************************

    static constexpr size_t min_nr_of_correspondences = 8;

    if (adapter_mono.getNumberCorrespondences() >= min_nr_of_correspondences) {
      if (!FLAGS_use_ransac) {
        // Without RANSAC [OPTIONAL]
        // Consider using the "extractPose" function provided to obtain inliers, rotation, and translation
        // Consider using "cv2Pose(Rmat, tmat)" to recover the rotation Rmat and translation tmat

        // ***************************** begin solution *****************************



       // ***************************** end solution *****************************
      } else {
        // (TODO) With RANSAC
        // Consider using ``eigen2Pose`` to convert the solution of ransac to ``relative_pose_estimate``


        // ***************************** begin solution *****************************

        opengv::sac::Ransac<RansacProblem> ransac;
        auto problem = std::make_shared<RansacProblem>(
            adapter_mono, RansacProblem::EIGHTPT, !FLAGS_fixed_seed);

        if (adapter_mono.getNumberCorrespondences() <
            static_cast<size_t>(problem->getSampleSize())) {
          ROS_WARN("Too few matches for EIGHTPT RANSAC.");
          save_current_frame();
          return;
        }

        ransac.sac_model_ = problem;
        ransac.threshold_ = 1e-5;
        ransac.max_iterations_ = 1000;

        if (!ransac.computeModel() || !ransac.model_coefficients_.allFinite()) {
          ROS_WARN("EIGHTPT RANSAC failed.");
          save_current_frame();
          return;
        }

        relative_pose_estimate = eigen2Pose(ransac.model_coefficients_);

        ROS_INFO_STREAM("EIGHTPT inliers: " << ransac.inliers_.size()
                        << "/" << bearing_vector_1.size());
        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to compute pose estimation using"
               " Longuet-Higgins' algorithm.");
      save_current_frame();
      return;
    }
    break;
  }
  case 2: {
    // *********************************************************************************
    // ******************************** 2-point Method *********************************
    // *********************************************************************************

    static constexpr size_t min_nr_of_correspondences = 2;
    if (adapter_mono.getNumberCorrespondences() >= min_nr_of_correspondences) {

      // Obtain the rotation part from ground-truth
      tf::Transform curr_frame, prev_frame;
      tf::poseMsgToTF(curr_pose_.pose, curr_frame);
      tf::poseMsgToTF(prev_pose_.pose, prev_frame);
      Eigen::Matrix3d rotation;
      tf::matrixTFToEigen(prev_frame.inverseTimes(curr_frame).getBasis(), rotation);
      adapter_mono.setR12(rotation);

      // We only estimate the translation part
      if (!FLAGS_use_ransac) {
        // Without RANSAC (OPTIONAL)
        // ***************************** begin solution *****************************



        // ***************************** end solution *****************************
      } else {
        // (TODO) With RANSAC
        // ***************************** begin solution *****************************

        opengv::sac::Ransac<RansacProblemGivenRot> ransac;
        auto problem = std::make_shared<RansacProblemGivenRot>(
            adapter_mono, !FLAGS_fixed_seed);

        if (adapter_mono.getNumberCorrespondences() <
            static_cast<size_t>(problem->getSampleSize())) {
          ROS_WARN("Too few matches for TWOPOINT RANSAC.");
          save_current_frame();
          return;
        }

        ransac.sac_model_ = problem;
        ransac.threshold_ = FLAGS_twopt_threshold;
        ransac.max_iterations_ = 1000;

        if (!ransac.computeModel() || !ransac.model_coefficients_.allFinite()) {
          ROS_WARN("TWOPOINT RANSAC failed.");
          save_current_frame();
          return;
        }

        opengv::transformation_t refined_model = ransac.model_coefficients_;

        if (ransac.inliers_.size() >= 3) {
          Eigen::MatrixXd A(ransac.inliers_.size(), 3);

          for (size_t k = 0; k < ransac.inliers_.size(); ++k) {
            const int i = ransac.inliers_[k];
            const Eigen::Vector3d normal =
                bearing_vector_1[i].cross(rotation * bearing_vector_2[i]);
            A.row(k) = normal.transpose();
          }

          Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
          const auto singular_values = svd.singularValues();

          if (singular_values(1) > 1e-10) {
            Eigen::Vector3d translation = svd.matrixV().col(2).normalized();

            // Keep the translation sign selected by RANSAC.
            if (translation.dot(ransac.model_coefficients_.col(3)) < 0.0) {
              translation = -translation;
            }

            refined_model.block<3, 3>(0, 0) = rotation;
            refined_model.col(3) = translation;
          }
        }

        if (!refined_model.allFinite()) {
          ROS_WARN("TWOPOINT refinement failed.");
          save_current_frame();
          return;
        }

        relative_pose_estimate = eigen2Pose(refined_model);

        ROS_INFO_STREAM("TWOPOINT inliers: " << ransac.inliers_.size()
                        << "/" << bearing_vector_1.size());

        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to estimate relative translation using 2pt algorithm.");
      save_current_frame();
      return;
    }
    break;
  }
  case 3: {
    // *********************************************************************************
    // ******************************** 3-point Method *********************************
    // *********************************************************************************

    // Scale the bearing vectors to point clouds, by querying the depth values of each keypoint
    if (prev_depth.type() != CV_32FC1 || depth.type() != CV_32FC1) {
      ROS_ERROR("Arun requires 32FC1 depth images.");
      save_current_frame();
      return;
    }

    opengv::points_t cloud_1, cloud_2;
    for (size_t i = 0; i < bearing_vector_1.size(); ++i) {
      const int x1 = static_cast<int>(std::floor(pts1[i].x));
      const int y1 = static_cast<int>(std::floor(pts1[i].y));
      const int x2 = static_cast<int>(std::floor(pts2[i].x));
      const int y2 = static_cast<int>(std::floor(pts2[i].y));

      if (x1 < 0 || x1 >= prev_depth.cols ||
          y1 < 0 || y1 >= prev_depth.rows ||
          x2 < 0 || x2 >= depth.cols ||
          y2 < 0 || y2 >= depth.rows) {
        continue;
      }

      const double d1 = prev_depth.at<float>(y1, x1);
      const double d2 = depth.at<float>(y2, x2);
      if (!std::isfinite(d1) || !std::isfinite(d2) ||
          d1 <= 0.0 || d2 <= 0.0) {
        continue;
      }

      const auto& b1 = bearing_vector_1[i];
      const auto& b2 = bearing_vector_2[i];
      if (!b1.allFinite() || !b2.allFinite() ||
          std::abs(b1.z()) < 1e-8 || std::abs(b2.z()) < 1e-8) {
        continue;
      }

      cloud_1.push_back(b1 * (d1 / b1.z()));
      cloud_2.push_back(b2 * (d2 / b2.z()));
    }

    Adapter3D adapter_3d(cloud_1, cloud_2);

    static constexpr int min_nr_of_correspondences = 3;
    if (adapter_3d.getNumberCorrespondences() >= min_nr_of_correspondences) {
      if (!FLAGS_use_ransac){
        // Without RANSAC (Optional)
        // ***************************** begin solution *****************************



        // ***************************** end solution *****************************
      } else{
        // (TODO) With RANSAC
        // ***************************** begin solution *****************************

        opengv::sac::Ransac<RansacProblem3D> ransac;
        auto problem = std::make_shared<RansacProblem3D>(adapter_3d, !FLAGS_fixed_seed);

        ransac.sac_model_ = problem;
        ransac.threshold_ = 0.05;  // 5 cm, assuming depth is in meters
        ransac.max_iterations_ = 1000;

        if (!ransac.computeModel() ||
            ransac.inliers_.size() < 3 ||
            !ransac.model_coefficients_.allFinite()) {
          ROS_WARN("ARUN RANSAC failed.");
          save_current_frame();
          return;
        }

        opengv::transformation_t refined_model;
        problem->optimizeModelCoefficients(
            ransac.inliers_, ransac.model_coefficients_, refined_model);

        if (!refined_model.allFinite()) {
          ROS_WARN("ARUN refinement failed.");
          save_current_frame();
          return;
        }

        relative_pose_estimate = eigen2Pose(refined_model);

        ROS_INFO_STREAM("ARUN inliers: " << ransac.inliers_.size()
                        << "/" << cloud_1.size());


        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to estimate absolute transform using Arun's 3pt algorithm.");
      save_current_frame();
      return;
    }
    break;
  }
  default: {
    ROS_ERROR("Wrong pose_estimator flag!");
  }
  }

  // scale the relative pose estimate (for 5-pt, 2-pt and 8-pt, the translation is up to scale)
  if (FLAGS_pose_estimator < 3) {
    scaleTranslation(relative_pose_estimate.position, prev_pose_, curr_pose_);
  }

  // pose estimate w.r.t. ground truth
  updatePoseEstimate(prev_pose_.pose, relative_pose_estimate, pose_estimation.pose);

  // Compute tf::Transform in order to do actual pose operations.
  tf::Transform gt_t_prev_frame, gt_t_curr_frame;
  tf::Transform est_t_prev_frame, est_t_curr_frame;
  tf::poseMsgToTF(pose_estimation.pose, est_t_curr_frame);
  tf::poseMsgToTF(curr_pose_.pose, gt_t_curr_frame);
  tf::poseMsgToTF(prev_pose_.pose, est_t_prev_frame);
  tf::poseMsgToTF(prev_pose_.pose, gt_t_prev_frame);

  // Evaluate pose errors
  evaluateRPE(gt_t_prev_frame, gt_t_curr_frame,
              est_t_prev_frame, est_t_curr_frame,
              rgb_msg->header.stamp);

  // (TODO) Now, publish your estimated pose here. Since your pose estimate is
  // relative to the previous frame, use the ground-truth pose of the previous
  // frame to publish your pose_estimate in world frame. This will allow you to
  // see how far away is your estimate from the ground-truth one in Rviz.
  // Remember to subscirbe to the right topic in Rviz. Make this abosolute pose
  pose_estimation.header.stamp = rgb_msg->header.stamp;
  pose_estimation.header.frame_id = "world";
  pub_pose_estimation_.publish(pose_estimation);

  // Save for next iteration
  prev_bgr = bgr.clone();
  prev_depth = depth.clone();
  prev_pose_ = curr_pose_;
  previous_image_stamp_ = rgb_msg->header.stamp;
}

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  if (FLAGS_cache_only && FLAGS_match_cache.empty()) {
    std::cerr << "--cache_only=true requires a new --match_cache=PREFIX" << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "keypoint_trackers");
  ros::NodeHandle local_nh("~");

  // populate camera intrinsics and distortion
  camera_params_.K = cv::Mat::zeros(3, 3, CV_64F);
  camera_params_.K.at<double>(0,0) = 415.69219381653056;
  camera_params_.K.at<double>(1,1) = 415.69219381653056;
  camera_params_.K.at<double>(0,2) = 360.0;
  camera_params_.K.at<double>(1,2) = 240.0;
  camera_params_.D = cv::Mat::zeros(cv::Size(5,1),CV_64F);

  // Constant extrinsics of the camera wrt the body, T_{camera}^{body}
  T_camera_body = cv::Mat::zeros(cv::Size(4,4),CV_64F);
  T_camera_body.at<double>(0,2) = 1.0;
  T_camera_body.at<double>(1,0) = -1.0;
  T_camera_body.at<double>(1,3) = 0.05;
  T_camera_body.at<double>(2,1) = -1.0;
  T_camera_body.at<double>(3,3) = 1.0;
  // convert to geometry_msgs::Pose
  R_camera_body = T_camera_body(cv::Range(0,3),cv::Range(0,3));
  t_camera_body = T_camera_body(cv::Range(0,3),cv::Range(3,4));
  pose_camera_body = cv2Pose(R_camera_body,t_camera_body);
  // convert to tf transform
  tf::poseMsgToTF(pose_camera_body, transform_camera_body);

  feature_tracker_.reset(new TrackerWrapper());

  // Subscribe to drone pose estimation.
  auto pose_sub = local_nh.subscribe("/ground_truth_pose", 10, poseCallbackTesse);

  // Subscribe to rgb and depth images.
  image_transport::ImageTransport it(local_nh);
  image_transport::SubscriberFilter sf_rgb(it, "/rgb_images_topic", 1);
  image_transport::SubscriberFilter sf_depth(it, "/depth_images_topic", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sf_rgb, sf_depth);
  sync.registerCallback(cameraCallback);

  // Advertise drone pose.
  pub_pose_gt_ = local_nh.advertise<geometry_msgs::PoseStamped>("/gt_camera_pose", 1);
  pub_pose_estimation_ = local_nh.advertise<geometry_msgs::PoseStamped>("/camera_pose", 1);

  while (ros::ok()) {
    ros::spinOnce();
    cv::waitKey(1);
  }
  cv::destroyAllWindows();

  return EXIT_SUCCESS;
}
