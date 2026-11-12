/* Dense Farneback optical flow for LAB5 Deliverable 8. */
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

class DenseFlowNode {
 public:
  explicit DenseFlowNode(ros::NodeHandle* nh)
      : transport_(*nh), private_nh_("~") {
    private_nh_.param("visualize", visualize_, false);
    private_nh_.param("save_pair", save_pair_, 1);
    private_nh_.param("output_dir", output_dir_, std::string("."));
    private_nh_.param("display_max_motion", display_max_motion_, 3.0);
    if (display_max_motion_ <= 0.0) display_max_motion_ = 3.0;
    subscriber_ = transport_.subscribe("/images_topic", 100,
                                      &DenseFlowNode::onImage, this);
  }

  void printSummary() const {
    std::cout << "Dense flow pairs: " << pairs_ << std::endl;
    if (pairs_ > 0) {
      std::cout << "Avg. mean motion (px/frame): "
                << motion_sum_ / pairs_ << std::endl;
    }
  }

 private:
  void onImage(const sensor_msgs::ImageConstPtr& msg) {
    try {
      const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      if (image.empty()) return;
      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      if (previous_gray_.empty() || previous_gray_.size() != gray.size()) {
        previous_gray_ = gray.clone();
        return;
      }

      cv::Mat flow;
      cv::calcOpticalFlowFarneback(previous_gray_, gray, flow,
                                  0.5, 3, 15, 3, 5, 1.2, 0);
      previous_gray_ = gray.clone();
      ++pairs_;

      std::vector<cv::Mat> components;
      cv::split(flow, components);
      cv::Mat magnitude, direction;
      cv::cartToPolar(components[0], components[1], magnitude, direction, true);
      const double mean_motion = cv::mean(magnitude)[0];
      motion_sum_ += mean_motion;

      if (pairs_ == static_cast<unsigned int>(save_pair_) || visualize_) {
        cv::Mat hue, saturation(magnitude.size(), CV_8U, cv::Scalar(255));
        cv::Mat value, hsv, colored;
        direction.convertTo(hue, CV_8U, 0.5);  // degrees / 2 -> HSV hue
        magnitude.convertTo(value, CV_8U, 255.0 / display_max_motion_);
        cv::merge(std::vector<cv::Mat>{hue, saturation, value}, hsv);
        cv::cvtColor(hsv, colored, cv::COLOR_HSV2BGR);

        if (pairs_ == static_cast<unsigned int>(save_pair_)) {
          cv::Mat arrows = image.clone();
          for (int y = 12; y < flow.rows; y += 20) {
            for (int x = 12; x < flow.cols; x += 20) {
              const cv::Point2f displacement = flow.at<cv::Point2f>(y, x);
              if (cv::norm(displacement) < 0.2) continue;
              const cv::Point start(x, y);
              const cv::Point end(
                  x + static_cast<int>(std::lround(4.0 * displacement.x)),
                  y + static_cast<int>(std::lround(4.0 * displacement.y)));
              cv::arrowedLine(arrows, start, end, cv::Scalar(0, 255, 0),
                              1, cv::LINE_AA, 0, 0.25);
            }
          }
          const std::string color_path = output_dir_ + "/dense_flow_hsv.png";
          const std::string arrow_path = output_dir_ + "/dense_flow_arrows.png";
          if (!cv::imwrite(color_path, colored) ||
              !cv::imwrite(arrow_path, arrows)) {
            ROS_ERROR("Could not save dense flow images to %s",
                      output_dir_.c_str());
          } else {
            double max_motion = 0.0;
            cv::minMaxLoc(magnitude, nullptr, &max_motion);
            ROS_INFO_STREAM("Saved dense flow pair " << pairs_ << " to "
                            << output_dir_ << "; mean motion " << mean_motion
                            << " px, max motion " << max_motion << " px");
          }
        }
        if (visualize_) {
          cv::imshow("Dense optical flow", colored);
          cv::waitKey(1);
        }
      }
      if (pairs_ % 100 == 0) {
        ROS_INFO_STREAM("Dense flow pairs processed: " << pairs_);
      }
    } catch (const cv_bridge::Exception& error) {
      ROS_ERROR("Could not convert image to bgr8: %s", error.what());
    } catch (const cv::Exception& error) {
      ROS_ERROR("Dense flow failed: %s", error.what());
    }
  }

  image_transport::ImageTransport transport_;
  ros::NodeHandle private_nh_;
  image_transport::Subscriber subscriber_;
  cv::Mat previous_gray_;
  bool visualize_ = false;
  int save_pair_ = 1;
  double display_max_motion_ = 3.0;
  std::string output_dir_;
  unsigned int pairs_ = 0;
  double motion_sum_ = 0.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dense_flow");
  ros::NodeHandle nh;
  DenseFlowNode node(&nh);
  ros::spin();
  node.printSummary();
  return 0;
}
