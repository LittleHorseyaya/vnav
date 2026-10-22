// MIT VNAV LAB4: minimum-snap position and continuous-yaw trajectories.
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseArray.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

class WaypointFollower {
  ros::Subscriber state_sub_, vertices_sub_;
  ros::Publisher desired_pub_;
  ros::Timer timer_;
  ros::NodeHandle pnh_{"~"};
  tf::TransformBroadcaster broadcaster_;
  Eigen::Vector3d x_{Eigen::Vector3d::Zero()};
  double current_yaw_ = 0;
  bool have_state_ = false;
  double v_max_ = 2.5, a_max_ = 1.5, time_scale_ = 1.5;
  ros::Time start_time_, last_tf_stamp_;
  mav_trajectory_generation::Trajectory trajectory_, yaw_trajectory_;

  // Part 1.1: read the current position before accepting any waypoints.
  void onCurrentState(const nav_msgs::Odometry& msg) {
    x_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
    current_yaw_ = tf::getYaw(msg.pose.pose.orientation);
    have_state_ = x_.allFinite() && std::isfinite(current_yaw_);
  }

  // Part 1.2: 3D position vertices, continuous yaw, common segment times.
  void generateOptimizedTrajectory(const geometry_msgs::PoseArray& msg) {
    if (!trajectory_.empty()) return;
    if (!have_state_ || msg.poses.empty()) {
      ROS_WARN_THROTTLE(2, "Waiting for current_state and nonempty waypoints.");
      return;
    }
    using namespace mav_trajectory_generation;
    using namespace mav_trajectory_generation::derivative_order;
    constexpr int D = 3, N = 10;
    Vertex::Vector vertices, yaw_vertices;
    Vertex start_position(D), start_yaw(1);
    start_position.makeStartOrEnd(x_, SNAP);
    Eigen::VectorXd initial_yaw(1);
    initial_yaw << current_yaw_;
    start_yaw.makeStartOrEnd(initial_yaw, SNAP);
    vertices.push_back(start_position);
    yaw_vertices.push_back(start_yaw);
    double last_yaw = current_yaw_;
    for (std::size_t i = 0; i < msg.poses.size(); ++i) {
      const auto& pose = msg.poses[i];
      Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);
      const double raw_yaw = tf::getYaw(pose.orientation);
      if (!p.allFinite() || !std::isfinite(raw_yaw)) {
        ROS_ERROR("Nonfinite waypoint, refusing to plan.");
        return;
      }
      const double yaw = last_yaw + std::atan2(std::sin(raw_yaw-last_yaw),
                                              std::cos(raw_yaw-last_yaw));
      Eigen::VectorXd y(1);
      y << yaw;
      Vertex position_vertex(D), yaw_vertex(1);
      if (i+1 == msg.poses.size()) {
        position_vertex.makeStartOrEnd(p, SNAP);
        yaw_vertex.makeStartOrEnd(y, SNAP);
      } else {
        position_vertex.addConstraint(POSITION, p);
        yaw_vertex.addConstraint(ORIENTATION, y);
      }
      vertices.push_back(position_vertex);
      yaw_vertices.push_back(yaw_vertex);
      last_yaw = yaw;
    }
    auto segment_times = estimateSegmentTimes(vertices, v_max_, a_max_);
    for (double& t : segment_times) t = std::max(0.5, time_scale_*t);
    PolynomialOptimization<N> position_opt(D), yaw_opt(1);
    if (!position_opt.setupFromVertices(vertices, segment_times, SNAP) ||
        !yaw_opt.setupFromVertices(yaw_vertices, segment_times, SNAP) ||
        !position_opt.solveLinear() || !yaw_opt.solveLinear()) {
      ROS_ERROR("Trajectory optimization failed.");
      return;
    }
    position_opt.getTrajectory(&trajectory_);
    yaw_opt.getTrajectory(&yaw_trajectory_);
    start_time_ = ros::Time::now();
    pnh_.setParam("trajectory_duration", trajectory_.getMaxTime());
    pnh_.setParam("gate_count", static_cast<int>(msg.poses.size()));
    pnh_.setParam("trajectory_start_time", start_time_.toSec());
    ROS_INFO("Generated trajectory through %zu gates; duration %.3f s.",
             msg.poses.size(), trajectory_.getMaxTime());
  }

  // Part 1.3: sample simulation elapsed time and hold the terminal state.
  void publishDesiredState(const ros::TimerEvent&) {
    if (trajectory_.empty()) return;
    using namespace mav_trajectory_generation::derivative_order;
    const ros::Time now = ros::Time::now();
    const double duration = trajectory_.getMaxTime();
    const double t = std::max(0.0, std::min((now-start_time_).toSec(), duration));
    const Eigen::Vector3d p = trajectory_.evaluate(t, POSITION);
    const Eigen::Vector3d v = trajectory_.evaluate(t, VELOCITY);
    const Eigen::Vector3d a = trajectory_.evaluate(t, ACCELERATION);
    const double yaw = yaw_trajectory_.evaluate(t, ORIENTATION)[0];
    trajectory_msgs::MultiDOFJointTrajectoryPoint next;
    next.time_from_start = ros::Duration(t);
    next.transforms.resize(1);
    next.velocities.resize(1);
    next.accelerations.resize(1);
    next.transforms[0].translation.x = p.x();
    next.transforms[0].translation.y = p.y();
    next.transforms[0].translation.z = p.z();
    next.transforms[0].rotation = tf::createQuaternionMsgFromYaw(yaw);
    next.velocities[0].linear.x = v.x();
    next.velocities[0].linear.y = v.y();
    next.velocities[0].linear.z = v.z();
    next.velocities[0].angular.z = yaw_trajectory_.evaluate(t, VELOCITY)[0];
    next.accelerations[0].linear.x = a.x();
    next.accelerations[0].linear.y = a.y();
    next.accelerations[0].linear.z = a.z();
    next.accelerations[0].angular.z = yaw_trajectory_.evaluate(t, ACCELERATION)[0];
    desired_pub_.publish(next);
    if (now != last_tf_stamp_) {
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
      transform.setRotation(tf::createQuaternionFromYaw(yaw));
      broadcaster_.sendTransform(tf::StampedTransform(transform, now, "world", "av-desired"));
      last_tf_stamp_ = now;
    }
    ROS_INFO_THROTTLE(5, "Trajectory progress: %.1f%%", 100.0*t/duration);
  }

 public:
  explicit WaypointFollower(ros::NodeHandle& nh) {
    pnh_.param("v_max", v_max_, v_max_);
    pnh_.param("a_max", a_max_, a_max_);
    pnh_.param("time_scale", time_scale_, time_scale_);
    if (!(v_max_ > 0 && a_max_ > 0 && time_scale_ > 0))
      throw std::invalid_argument("v_max, a_max, time_scale must be positive");
    state_sub_ = nh.subscribe("/current_state", 1, &WaypointFollower::onCurrentState, this);
    vertices_sub_ = nh.subscribe("/desired_traj_vertices", 1,
                               &WaypointFollower::generateOptimizedTrajectory, this);
    desired_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 1);
    timer_ = nh.createTimer(ros::Duration(0.01), &WaypointFollower::publishDesiredState, this);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generation_node");
  ros::NodeHandle nh;
  WaypointFollower follower(nh);
  ros::spin();
  return 0;
}
