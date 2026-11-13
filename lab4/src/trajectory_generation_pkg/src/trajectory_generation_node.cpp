// MIT VNAV LAB4: minimum-snap position and continuous-yaw trajectories.
#include <algorithm>
#include "lab4_attitude_ff.h"
#include <vector>
#include <string>
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
  ros::Publisher desired_pub_, attitude_ff_pub_;
  ros::Timer timer_;
  ros::NodeHandle pnh_{"~"};
  tf::TransformBroadcaster broadcaster_;
  Eigen::Vector3d x_{Eigen::Vector3d::Zero()};
  double current_yaw_ = 0;
  bool have_state_ = false;
  double v_max_ = 2.5, a_max_ = 1.5, time_scale_ = 1.5;
  ros::Time start_time_, last_tf_stamp_;

  // PATH_CLOCK_EXPERIMENT_V1
  double path_speedup_ = 1.0, path_ramp_ = 1.0;
  double path_start_ = 0.0, execution_duration_ = 0.0;
  double attitude_preview_ = 0.0;

  // The phase rate ramps with a seventh-order smoothstep. Its first three
  // derivatives vanish at both ramp boundaries, so phase is C4 continuous.
  static void addPathRamp(double e, double start, double duration, double b,
                          double& phase, double& h1, double& h2,
                 double& h3, double& h4) {
    if (b==0.0 || e<=start) return;
    const double x=(e-start)/duration;
    if (x>=1.0) {
      phase+=b*(e-start-0.5*duration);
      h1+=b;
      return;
    }
    const double x2=x*x, x3=x2*x, x4=x3*x;
    const double x5=x4*x, x6=x5*x, x7=x6*x, x8=x7*x;
    const double S=35*x4-84*x5+70*x6-20*x7;
    const double I=7*x5-14*x6+10*x7-2.5*x8;
    const double dS=140*x3-420*x4+420*x5-140*x6;
    const double ddS=420*x2-1680*x3+2100*x4-840*x5;
    const double dddS=840*x-5040*x2+8400*x3-4200*x4;
    phase+=b*duration*I;
    h1+=b*S;
    h2+=b*dS/duration;
    h3+=b*ddS/(duration*duration);
    h4+=b*dddS/(duration*duration*duration);
  }

  double late_speedup_ = 0.0, late_start_ = -1.0;
  int late_after_gate_ = 8;
  int late_until_gate_ = 0;
  double late_end_ = -1.0;
  double late_return_ramp_ = 0.5;
  double late_final_rate_ = 0.0;
  double turn_slowdown_ = 0.0;
  double turn_start_ = -1.0, turn_end_ = -1.0;
  static constexpr double turn_ramp_ = 0.6;
  int turn_after_gate_ = 4, turn_until_gate_ = 7;

  void basePathClock(double elapsed, double& phase, double& h1, double& h2,
                 double& h3, double& h4) const {
    const double e = std::max(0.0, elapsed);
    phase = e;
    h1 = 1.0;
    h2 = h3 = h4 = 0.0;
    addPathRamp(e, path_start_, path_ramp_, path_speedup_ - 1.0, phase, h1, h2, h3, h4);
    if (turn_start_ >= 0.0)
      addPathRamp(e, turn_start_, turn_ramp_, -turn_slowdown_, phase, h1, h2, h3, h4);
    if (turn_end_ >= 0.0)
      addPathRamp(e, turn_end_, turn_ramp_, turn_slowdown_, phase, h1, h2, h3, h4);
    if (late_start_ >= 0.0)
      addPathRamp(e, late_start_, path_ramp_, late_speedup_, phase, h1, h2, h3, h4);
    if (late_end_ >= 0.0) {
      const double final_rate = late_final_rate_ > 0.0 ? late_final_rate_ : path_speedup_;
      addPathRamp(e, late_end_, late_return_ramp_, final_rate - (path_speedup_ + late_speedup_),
                  phase, h1, h2, h3, h4);
    }
  }

  // GATE_WINDOW_RETIME_V1. Compose a C4 local time warp with the base clock.
  bool gate_window_retime_ = false;
  double window_start_ = 0.0, window_duration_ = 0.0, window_gain_ = 0.0;

  void pathClock(double elapsed, double& phase, double& h1, double& h2,
                 double& h3, double& h4) const {
    double e=elapsed, d1=1.0, d2=0.0, d3=0.0, d4=0.0;
    if (window_gain_ > 0.0) {
      const double ramp=0.4*window_duration_;
      addPathRamp(elapsed, window_start_, ramp, window_gain_, e, d1, d2, d3, d4);
      addPathRamp(elapsed, window_start_+window_duration_-ramp, ramp, -window_gain_,
                  e, d1, d2, d3, d4);
    }
    basePathClock(e,phase,h1,h2,h3,h4);
    if (window_gain_ == 0.0) return;
    h4=h4*d1*d1*d1*d1+6.0*h3*d1*d1*d2+3.0*h2*d2*d2+4.0*h2*d1*d3+h1*d4;
    h3=h3*d1*d1*d1+3.0*h2*d1*d2+h1*d3;
    h2=h2*d1*d1+h1*d2;
    h1*=d1;
  }

  Eigen::Vector4d windowPeaks(double duration) const {
    using namespace mav_trajectory_generation::derivative_order;
    Eigen::Vector4d peaks=Eigen::Vector4d::Zero();
    for (int i=0; i<=300; ++i) {
      double p,h1,h2,h3,h4;
      pathClock(window_start_+duration*i/300.0,p,h1,h2,h3,h4);
      const Eigen::Vector3d v=trajectory_.evaluate(p,VELOCITY);
      const Eigen::Vector3d a=trajectory_.evaluate(p,ACCELERATION);
      const Eigen::Vector3d j=trajectory_.evaluate(p,JERK);
      const Eigen::Vector3d s=trajectory_.evaluate(p,SNAP);
      Eigen::Vector4d values;
      values << (v*h1).norm(), (a*h1*h1+v*h2+Eigen::Vector3d(0,0,9.81)).norm(),
          (j*h1*h1*h1+3.0*a*h1*h2+v*h3).norm(),
          (s*h1*h1*h1*h1+6.0*j*h1*h1*h2+3.0*a*h2*h2+4.0*a*h1*h3+v*h4).norm();
      if (!values.allFinite()) return Eigen::Vector4d::Constant(1e30);
      peaks=peaks.cwiseMax(values);
    }
    return peaks;
  }

  void planGateWindow(const std::vector<double>& segment_times, size_t gate_count) {
    if (!gate_window_retime_) return;
    double learning_gain=0.0;
    ros::param::get("/controller_node/learning_gain", learning_gain);
    if (gate_count != 12 || segment_times.size() < 12 || learning_gain != 0.0 ||
        attitude_preview_ != 0.0 || turn_slowdown_ != 0.0 ||
        late_after_gate_ != 7 || late_until_gate_ != 10 || late_speedup_ <= 0.0) {
      ROS_WARN("GATE WINDOW RETIME skipped: requires 12 gates and the gate 7-10 baseline clock");
      return;
    }
    double p=0.0;
    for (int i=0; i<7; ++i) p+=segment_times[i];
    window_start_=executionForPhase(p);
    for (int i=7; i<10; ++i) p+=segment_times[i];
    const double baseline_duration=executionForPhase(p)-window_start_;
    if (baseline_duration < 1.0) {
      ROS_WARN("GATE WINDOW RETIME skipped: window is shorter than 1 s");
      return;
    }
    const Eigen::Vector4d baseline=windowPeaks(baseline_duration);
    Eigen::Vector4d limits;
    limits << std::max(v_max_, baseline[0]), 1.10*baseline[1],
        1.20*baseline[2], 1.30*baseline[3];
    for (int step=20; step>=1; --step) {
      window_gain_=0.01*step;
      window_duration_=baseline_duration/(1.0+0.6*window_gain_);
      const Eigen::Vector4d candidate=windowPeaks(window_duration_);
      if ((candidate.array() <= limits.array()).all() && baseline.maxCoeff() < 1e29) {
        ROS_INFO("GATE WINDOW RETIME accepted: gain=%.2f, reference saving=%.3f s, thrust/m=%.2f -> %.2f m/s^2",
                 window_gain_,baseline_duration-window_duration_,baseline[1],candidate[1]);
        return;
      }
    }
    window_gain_=0.0;
    ROS_WARN("GATE WINDOW RETIME kept baseline: all trial rates exceeded sampled limits");
  }

  double executionForPhase(double goal) const {
    if ((path_speedup_==1.0 && late_start_<0.0 && turn_start_<0.0) || goal<=path_start_) return goal;
    // The phase derivative is at least 1 - turn_slowdown_, even during ramps.
    double low=0.0, high=goal/(1.0-turn_slowdown_);
    for (int k=0;k<60;++k) {
      const double middle=0.5*(low+high);
      double phase,h1,h2,h3,h4;
      pathClock(middle,phase,h1,h2,h3,h4);
      if (phase<goal) low=middle; else high=middle;
    }
    return 0.5*(low+high);
  }

  mav_trajectory_generation::Trajectory trajectory_, yaw_trajectory_;

  // Part 1.1: read the current position before accepting any waypoints.
  void onCurrentState(const nav_msgs::Odometry& msg) {
    x_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
    current_yaw_ = tf::getYaw(msg.pose.pose.orientation);
    have_state_ = x_.allFinite() && std::isfinite(current_yaw_);
  }

  // Part 1.2: Straight-line trajectory with trapezoidal velocity profile.
  void generateOptimizedTrajectory(const geometry_msgs::PoseArray& msg) {
    if (!trajectory_.empty()) return;
    if (!have_state_ || msg.poses.empty()) {
      ROS_WARN_THROTTLE(2, "Waiting for current_state and nonempty waypoints.");
      return;
    }
    using namespace mav_trajectory_generation;
    using namespace mav_trajectory_generation::derivative_order;
    constexpr int D = 3, N = 10;
    
    // Build waypoints (start + all gates)
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(x_);  // Current position as start
    for (const auto& pose : msg.poses) {
      waypoints.push_back(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
    }
    

    // FINISH_EXTENSION_EXPERIMENT_V1
    double finish_extension = 0.0;
    pnh_.param("finish_extension", finish_extension, 0.0);
    if (!std::isfinite(finish_extension) || finish_extension < 0.0 || finish_extension > 20.0) {
      ROS_ERROR("finish_extension must be finite and in [0,20] metres");
      return;
    }
    if (finish_extension > 0.0) {
      if (waypoints.size() < 3) {
        ROS_ERROR("Finish extension requires at least two gate positions");
        return;
      }
      Eigen::Vector3d heading=waypoints.back()-waypoints[waypoints.size()-2];
      heading.z()=0.0;
      const double length=heading.norm();
      if (!heading.allFinite() || !std::isfinite(length) || length < 1e-5) {
        ROS_ERROR("Cannot define horizontal finish direction");
        return;
      }
      const Eigen::Vector3d stop_point=waypoints.back()+finish_extension*heading/length;
      if (!stop_point.allFinite()) {
        ROS_ERROR("Nonfinite finish stop point");
        return;
      }
      waypoints.push_back(stop_point);
      ROS_INFO("FINISH EXTENSION: %.2f m; stop point [%.3f, %.3f, %.3f]; all original gates retained",
               finish_extension,stop_point.x(),stop_point.y(),stop_point.z());
    }

    // Straight-line segments with trapezoidal velocity profile
    Vertex::Vector vertices;
    Vertex start_position(D);
    start_position.makeStartOrEnd(waypoints[0], SNAP);
    vertices.push_back(start_position);
    
    std::vector<double> segment_times;
    
    for (size_t i = 1; i < waypoints.size(); ++i) {
      Eigen::Vector3d dist = waypoints[i] - waypoints[i-1];
      double d = dist.norm();
      
      // Trapezoidal velocity profile calculation
      // Acceleration distance: d_acc = v_max^2 / (2*a_max)
      double d_acc = v_max_ * v_max_ / (2.0 * a_max_);
      
      double t_seg;
      if (d >= 2.0 * d_acc) {
        // Has constant velocity segment: accelerate + constant + decelerate
        double d_const = d - 2.0 * d_acc;
        double t_acc = v_max_ / a_max_;
        double t_const = d_const / v_max_;
        t_seg = 2.0 * t_acc + t_const;
      } else {
        // Triangular velocity profile: only accelerate and decelerate
        double v_peak = std::sqrt(a_max_ * d);
        t_seg = 2.0 * v_peak / a_max_;
      }
      
      // Apply time scaling
      t_seg = std::max(0.3, time_scale_ * t_seg);
      segment_times.push_back(t_seg);
      
      // Add intermediate waypoint (straight-line constraint)
      if (i < waypoints.size() - 1) {
        Vertex position_vertex(D);
        position_vertex.addConstraint(POSITION, waypoints[i]);
        vertices.push_back(position_vertex);
      } else {
        // Final waypoint
        Vertex end_position(D);
        end_position.makeStartOrEnd(waypoints[i], SNAP);
        vertices.push_back(end_position);
      }
    }
    

    // CONTINUOUS_TIMING_EXPERIMENT_V1
    std::string timing_mode;
    pnh_.param("timing_mode", timing_mode, std::string("legacy"));
    double lateral_accel = 8.0;
    pnh_.param("lateral_accel", lateral_accel, 8.0);
    if (timing_mode != "legacy" && timing_mode != "continuous") {
      ROS_ERROR("timing_mode must be legacy or continuous");
      return;
    }
    if (timing_mode == "continuous") {
      if (!std::isfinite(lateral_accel) || lateral_accel <= 0.0 ||
          !std::isfinite(v_max_) || !std::isfinite(a_max_) ||
          !std::isfinite(time_scale_)) {
        ROS_ERROR("Invalid continuous timing parameters");
        return;
      }
      const size_t count = waypoints.size() - 1;
      std::vector<double> lengths(count);
      std::vector<double> speeds(count + 1, v_max_);
      for (size_t k = 0; k < count; ++k) {
        lengths[k] = (waypoints[k+1] - waypoints[k]).norm();
        if (!std::isfinite(lengths[k]) || lengths[k] < 1e-5) {
          ROS_ERROR("Nonfinite or duplicate waypoint at segment %zu", k);
          return;
        }
      }
      speeds.front() = 0.0;
      speeds.back() = 0.0;
      // A corner heuristic for time estimation, not a certified curvature bound.
      for (size_t k = 1; k < count; ++k) {
        const Eigen::Vector3d incoming =
            (waypoints[k] - waypoints[k-1]) / lengths[k-1];
        const Eigen::Vector3d outgoing =
            (waypoints[k+1] - waypoints[k]) / lengths[k];
        const double cosine = std::max(-1.0, std::min(1.0, incoming.dot(outgoing)));
        const double sin_half = std::sqrt(std::max(0.0, (1.0-cosine)*0.5));
        if (sin_half > 1e-3) {
          const double radius = std::min(lengths[k-1], lengths[k]) / (2*sin_half);
          speeds[k] = std::min(v_max_, std::sqrt(lateral_accel * radius));
        }
      }
      // Forward acceleration pass; backward braking pass.
      for (size_t k = 0; k < count; ++k)
        speeds[k+1] = std::min(speeds[k+1],
            std::sqrt(speeds[k]*speeds[k] + 2*a_max_*lengths[k]));
      for (size_t k = count; k > 0; --k)
        speeds[k-1] = std::min(speeds[k-1],
            std::sqrt(speeds[k]*speeds[k] + 2*a_max_*lengths[k-1]));
      segment_times.clear();
      for (size_t k = 0; k < count; ++k) {
        const double u = speeds[k], w = speeds[k+1];
        const double peak = std::min(v_max_,
            std::sqrt(a_max_*lengths[k] + 0.5*(u*u+w*w)));
        const double ramp_distance = (2*peak*peak-u*u-w*w)/(2*a_max_);
        const double cruise_distance = std::max(0.0, lengths[k]-ramp_distance);
        const double raw_time = (2*peak-u-w)/a_max_ + cruise_distance/peak;
        const double allocated_time = std::max(0.3, time_scale_*raw_time);
        if (!std::isfinite(allocated_time) || allocated_time <= 0.0) {
          ROS_ERROR("Invalid segment duration");
          return;
        }
        segment_times.push_back(allocated_time);
        ROS_INFO("Timing segment %zu: length=%.3f, estimated entry/exit=%.3f/%.3f, T=%.3f",
                 k, lengths[k], u, w, allocated_time);
      }
    }
    ROS_INFO("Timing mode: %s", timing_mode.c_str());


    // LOCAL_TIMING_EXPERIMENT_V1
    std::vector<double> local_time_scales;
    if (pnh_.hasParam("segment_time_scales") &&
        !pnh_.getParam("segment_time_scales", local_time_scales)) {
      ROS_ERROR("segment_time_scales must be a numeric array");
      return;
    }
    if (!local_time_scales.empty()) {
      if (local_time_scales.size() != segment_times.size()) {
        ROS_ERROR("segment_time_scales requires %zu entries", segment_times.size());
        return;
      }
      for (size_t k = 0; k < local_time_scales.size(); ++k) {
        if (!std::isfinite(local_time_scales[k]) || local_time_scales[k] <= 0.0) {
          ROS_ERROR("Invalid local time multiplier at segment %zu", k);
          return;
        }
      }
      for (size_t k = 0; k < segment_times.size(); ++k) {
        segment_times[k] = std::max(0.3, segment_times[k]*local_time_scales[k]);
        if (!std::isfinite(segment_times[k])) {
          ROS_ERROR("Nonfinite local segment duration");
          return;
        }
        ROS_INFO("LOCAL TIMING segment %zu: factor=%.3f, final T=%.4f s",
                 k, local_time_scales[k], segment_times[k]);
      }
    }


    // CORNER_VELOCITY_EXPERIMENT_V1
    int corner_gate = 0;
    double corner_speed = 4.0;
    pnh_.param("corner_gate", corner_gate, 0);
    pnh_.param("corner_speed", corner_speed, 4.0);
    if (corner_gate != 0) {
      // User numbering starts at 1; vertex 0 is the initial position.
      if (corner_gate < 1 ||
          static_cast<size_t>(corner_gate) >= waypoints.size()-1 ||
          !std::isfinite(corner_speed) || corner_speed <= 0.0) {
        ROS_ERROR("corner_gate must identify an intermediate gate; corner_speed must be positive");
        return;
      }
      const size_t k = static_cast<size_t>(corner_gate);
      Eigen::Vector3d outgoing = waypoints[k+1] - waypoints[k];
      const double length = outgoing.norm();
      if (!outgoing.allFinite() || !std::isfinite(length) || length < 1e-5) {
        ROS_ERROR("Cannot define outgoing direction at the selected gate");
        return;
      }
      // Shape the reference velocity toward the next gate before this crossing.
      // This is not an actual-state speed bound or a gate-aperture guarantee.
      const Eigen::Vector3d corner_velocity = corner_speed * outgoing / length;
      vertices[k].addConstraint(VELOCITY, corner_velocity);
      ROS_INFO("CORNER VELOCITY gate %d: [%.3f, %.3f, %.3f] m/s",
               corner_gate, corner_velocity.x(), corner_velocity.y(), corner_velocity.z());
    }

    // Generate trajectory
    PolynomialOptimization<N> position_opt(D);
    if (!position_opt.setupFromVertices(vertices, segment_times, SNAP) ||
        !position_opt.solveLinear()) {
      ROS_ERROR("Trajectory optimization failed.");
      return;
    }
    position_opt.getTrajectory(&trajectory_);

    // FIRST_GATE_VERTICAL_LIMIT
    double first_gate_vz = -1.0;
    pnh_.param("first_gate_vz", first_gate_vz, -1.0);
    if (std::isfinite(first_gate_vz) && first_gate_vz >= 0.0 &&
        vertices.size() > 2) {
      Eigen::Vector3d gate_v =
          trajectory_.evaluate(segment_times.front(), VELOCITY);
      gate_v.z() = std::min(gate_v.z(), first_gate_vz);
      vertices[1].addConstraint(VELOCITY, gate_v);
      if (!position_opt.setupFromVertices(vertices, segment_times, SNAP) ||
          !position_opt.solveLinear()) {
        ROS_ERROR("First-gate trajectory optimization failed.");
        trajectory_ = mav_trajectory_generation::Trajectory();
        return;
      }
      position_opt.getTrajectory(&trajectory_);
      ROS_INFO("First gate vertical velocity: %.3f m/s", gate_v.z());
    }

    


    // BOUNDED_RETIMER_EXPERIMENT_V1
    // RETIME_SEARCH_PASSES_V1
    int retime_passes = 8;
    pnh_.param("retime_passes", retime_passes, 8);
    retime_passes = std::max(1, std::min(30, retime_passes));
    bool retime_enabled = false;
    double retime_ratio = 0.95;
    pnh_.param("retime", retime_enabled, false);
    pnh_.param("retime_ratio", retime_ratio, 0.95);
    if (retime_enabled) {
      if (!std::isfinite(retime_ratio) || retime_ratio < 0.8 || retime_ratio > 1.0 ||
          segment_times.size() < 3) {
        ROS_WARN("Retimer disabled: ratio must be in [0.8,1] and at least 3 segments are required");
      } else {
        const std::vector<double> original_times = segment_times;
        const Trajectory original_trajectory = trajectory_;
        Segment::Vector baseline_segments;
        original_trajectory.getSegments(&baseline_segments);
        const std::vector<int> axes{0,1,2};
        // Exact polynomial extrema, not the scalar speed estimates used for timing.
        auto measure = [&](const Trajectory& traj, std::vector<double>* peaks)->bool {
          peaks->clear();
          for (int order=VELOCITY; order<=SNAP; ++order) {
            Extremum low, high;
            if (!traj.computeMinMaxMagnitude(order,axes,&low,&high) ||
                !std::isfinite(high.value) || high.value < 0.0) return false;
            peaks->push_back(high.value);
          }
          return true;
        };
        std::vector<double> baseline_peaks;
        bool valid = measure(original_trajectory,&baseline_peaks);
        double baseline_tilt = 0.0;
        for (size_t k=2; k<baseline_segments.size() && valid; ++k) {
          for (int sample=0; sample<=40; ++sample) {
            const double t=baseline_segments[k].getTime()*sample/40.0;
            Eigen::Vector3d force=baseline_segments[k].evaluate(t,ACCELERATION);
            force.z()+=9.81;
            if (!force.allFinite() || force.norm()<1e-6) { valid=false; break; }
            const double cosine=std::max(-1.0,std::min(1.0,force.z()/force.norm()));
            baseline_tilt=std::max(baseline_tilt,std::acos(cosine));
          }
        }
        if (!valid) {
          ROS_WARN("Retimer could not validate baseline; retaining original trajectory");
        } else {
          // Fix p/v/a/jerk/snap at nodes 1 and 2, with their original durations.
          // This freezes both initial polynomials, rather than only their times.
          Vertex::Vector fixed_vertices=vertices;
          double boundary=0.0;
          for (size_t k=1; k<=2; ++k) {
            boundary+=original_times[k-1];
            for (int order=VELOCITY; order<=SNAP; ++order)
              fixed_vertices[k].addConstraint(order,original_trajectory.evaluate(boundary,order));
          }
          const double factors[4]={1.10,1.10,1.15,1.20};
          const double tilt_allowance=3.0*std::acos(-1.0)/180.0;
          const double target=original_trajectory.getMaxTime()*retime_ratio;
          int total_accepted=0;
          for (int pass=0; pass<retime_passes && trajectory_.getMaxTime()>target; ++pass) {
            int accepted=0;
            for (size_t k=2; k<segment_times.size(); ++k) {
              std::vector<double> trial_times=segment_times;
              trial_times[k]*=0.98;
              if (trial_times[k]<std::max(0.3,0.8*original_times[k])) continue;
              PolynomialOptimization<N> trial_opt(D);
              if (!trial_opt.setupFromVertices(fixed_vertices,trial_times,SNAP) ||
                  !trial_opt.solveLinear()) continue;
              Trajectory trial;
              trial_opt.getTrajectory(&trial);
              std::vector<double> peaks;
              if (!measure(trial,&peaks)) continue;
              bool feasible=true;
              for (size_t j=0; j<peaks.size(); ++j) {
                if (peaks[j]>baseline_peaks[j]*factors[j]+1e-6) { feasible=false; break; }
              }
              if (!feasible) continue;
              Segment::Vector trial_segments;
              trial.getSegments(&trial_segments);
              if (trial_segments.size()!=baseline_segments.size()) continue;
              // Sampled shape and nominal attitude screens. These are not
              // obstacle-clearance or actuator-feasibility certificates.
              for (size_t j=0; j<trial_segments.size() && feasible; ++j) {
                for (int sample=0; sample<=40; ++sample) {
                  const double u=sample/40.0;
                  const Eigen::Vector3d pos=trial_segments[j].evaluate(u*trial_times[j],POSITION);
                  const Eigen::Vector3d old=baseline_segments[j].evaluate(u*original_times[j],POSITION);
                  const double deviation=(pos-old).norm();
                  const double allowed=(j<2) ? 1e-5 : 0.25;
                  if (!pos.allFinite() || !std::isfinite(deviation) || deviation>allowed) {
                    feasible=false; break;
                  }
                  if (j>=2) {
                    Eigen::Vector3d force=trial_segments[j].evaluate(u*trial_times[j],ACCELERATION);
                    force.z()+=9.81;
                    if (!force.allFinite() || force.norm()<1e-6) { feasible=false; break; }
                    const double cosine=std::max(-1.0,std::min(1.0,force.z()/force.norm()));
                    if (std::acos(cosine)>baseline_tilt+tilt_allowance) { feasible=false; break; }
                  }
                }
              }
              if (!feasible) continue;
              trajectory_=trial;
              segment_times=trial_times;
              ++accepted; ++total_accepted;
              if (trajectory_.getMaxTime()<=target) break;
            }
            if (accepted==0) break;
          }
          ROS_INFO("RETIME: original=%.4f s, final=%.4f s, accepted=%d; first 2 segments frozen",
                   original_trajectory.getMaxTime(),trajectory_.getMaxTime(),total_accepted);
          for (size_t k=0; k<segment_times.size(); ++k)
            ROS_INFO("RETIMED segment %zu: factor=%.4f, T=%.4f s",
                     k,segment_times[k]/original_times[k],segment_times[k]);
        }
      }
    }

    // Validate the actual polynomial, after the first-gate modification.
    double actual_peak_v = 0.0, actual_peak_a = 0.0;
    if (!trajectory_.computeMaxVelocityAndAcceleration(&actual_peak_v, &actual_peak_a) ||
        !std::isfinite(actual_peak_v) || !std::isfinite(actual_peak_a)) {
      ROS_ERROR("Trajectory extrema validation failed");
      trajectory_ = mav_trajectory_generation::Trajectory();
      return;
    }
    ROS_INFO("ACTUAL POLYNOMIAL: T=%.4f s, peak_v=%.4f m/s, peak_a=%.4f m/s^2",
             trajectory_.getMaxTime(), actual_peak_v, actual_peak_a);

    // Yaw trajectory (keep initial orientation)
    Vertex::Vector yaw_vertices;
    Vertex start_yaw(1);
    Eigen::VectorXd initial_yaw(1);
    initial_yaw << current_yaw_;
    start_yaw.makeStartOrEnd(initial_yaw, SNAP);
    yaw_vertices.push_back(start_yaw);
    for (size_t i = 1; i < waypoints.size(); ++i) {
      Vertex yaw_vertex(1);
      if (i == waypoints.size() - 1) {
        yaw_vertex.makeStartOrEnd(initial_yaw, SNAP);
      } else {
        yaw_vertex.addConstraint(ORIENTATION, initial_yaw);
      }
      yaw_vertices.push_back(yaw_vertex);
    }
    PolynomialOptimization<N> yaw_opt(1);
    yaw_opt.setupFromVertices(yaw_vertices, segment_times, SNAP);
    yaw_opt.solveLinear();
    yaw_opt.getTrajectory(&yaw_trajectory_);
    


    // Freeze physical timing up to the second gate; only the path clock changes
    // afterwards. The geometric polynomial itself is not solved again here.
    path_start_=0.0;
    for (size_t k=0;k<std::min<size_t>(2,segment_times.size());++k)
      path_start_+=segment_times[k];
    window_gain_=0.0;
    late_start_ = late_end_ = -1.0;
    turn_start_ = turn_end_ = -1.0;
    if (turn_slowdown_ > 0.0) {
      double learning_gain = 0.0;
      ros::param::get("/controller_node/learning_gain", learning_gain);
      if (static_cast<size_t>(turn_until_gate_) >= msg.poses.size() ||
          (late_speedup_ > 0.0 && late_after_gate_ < turn_until_gate_) || learning_gain > 0.0) {
        ROS_ERROR("Turn slowdown requires an intermediate end gate, learning_gain=0 and late boost at or after the end gate");
        trajectory_ = mav_trajectory_generation::Trajectory();
        return;
      }
      double gate_phase = 0.0;
      for (int k = 0; k < turn_after_gate_; ++k) gate_phase += segment_times[k];
      turn_start_ = executionForPhase(gate_phase);
      for (int k = turn_after_gate_; k < turn_until_gate_; ++k) gate_phase += segment_times[k];
      turn_end_ = executionForPhase(gate_phase);
      if (turn_start_ < path_start_ + path_ramp_ || turn_end_ < turn_start_ + turn_ramp_) {
        ROS_ERROR("Turn slowdown gates are too close to the initial ramp or each other");
        trajectory_ = mav_trajectory_generation::Trajectory();
        return;
      }
      ROS_INFO("TURN SLOWDOWN: reference gates %d to %d, start %.4f s, recovery %.4f s, rate %.3f",
               turn_after_gate_, turn_until_gate_, turn_start_, turn_end_, path_speedup_-turn_slowdown_);
    }
    if (late_speedup_ > 0.0) {
      double learning_gain = 0.0;
      ros::param::get("/controller_node/learning_gain", learning_gain);
      if (static_cast<size_t>(late_after_gate_) >= msg.poses.size() || learning_gain > 0.0) {
        ROS_ERROR("Late speedup requires an intermediate gate and learning_gain=0");
        trajectory_ = mav_trajectory_generation::Trajectory();
        return;
      }
      double gate_phase = 0.0;
      for (int k = 0; k < late_after_gate_; ++k) gate_phase += segment_times[k];
      late_start_ = executionForPhase(gate_phase);
      if (late_until_gate_ > 0) {
        if (late_until_gate_ <= late_after_gate_ ||
            static_cast<size_t>(late_until_gate_) >= msg.poses.size()) {
          ROS_ERROR("Late speed window requires start gate < return gate < final gate");
          trajectory_ = mav_trajectory_generation::Trajectory();
          return;
        }
        for (int k = late_after_gate_; k < late_until_gate_; ++k)
          gate_phase += segment_times[k];
        const double return_start = executionForPhase(gate_phase);
        if (return_start < late_start_ + path_ramp_) {
          ROS_ERROR("Return ramp would start before acceleration ramp ends");
          trajectory_ = mav_trajectory_generation::Trajectory();
          return;
        }
        late_end_ = return_start;
        ROS_INFO("LATE RETURN: reference gate %d, start %.4f s, ramp %.3f s",
                 late_until_gate_, late_end_, late_return_ramp_);
      }
      ROS_INFO("LATE CLOCK: reference gate %d, start %.4f s, speed %.3f -> %.3f",
               late_after_gate_, late_start_, path_speedup_, path_speedup_ + late_speedup_);
    }
    planGateWindow(segment_times,msg.poses.size());
    execution_duration_=executionForPhase(trajectory_.getMaxTime());
    double sampled_v=0.0, sampled_a=0.0;
    for (int sample=0;sample<=2000;++sample) {
      const double elapsed=execution_duration_*sample/2000.0;
      double phase,h1,h2,h3,h4;
      pathClock(elapsed,phase,h1,h2,h3,h4);
      phase=std::max(0.0,std::min(phase,trajectory_.getMaxTime()));
      const Eigen::Vector3d raw_v=trajectory_.evaluate(phase,VELOCITY);
      const Eigen::Vector3d raw_a=trajectory_.evaluate(phase,ACCELERATION);
      sampled_v=std::max(sampled_v,(raw_v*h1).norm());
      sampled_a=std::max(sampled_a,(raw_a*h1*h1+raw_v*h2).norm());
    }
    pnh_.setParam("geometric_duration",trajectory_.getMaxTime());
    pnh_.setParam("path_clock_prefix",path_start_);
    ROS_INFO("PATH CLOCK: speedup=%.3f, ramp=%.3f s, protected prefix=%.4f s, execution=%.4f s, sampled peak_v=%.4f, peak_a=%.4f",
             path_speedup_,path_ramp_,path_start_,execution_duration_,sampled_v,sampled_a);

    double planned_last_gate_time=0.0;
    for (size_t k=0; k<msg.poses.size() && k<segment_times.size(); ++k)
      planned_last_gate_time+=segment_times[k];
    planned_last_gate_time=executionForPhase(planned_last_gate_time);
    pnh_.setParam("planned_last_gate_time",planned_last_gate_time);
    ROS_INFO("FINISH TIMING: planned last gate %.4f s; final stop %.4f s",
             planned_last_gate_time,execution_duration_);

    start_time_ = ros::Time::now();
    pnh_.setParam("trajectory_duration", execution_duration_);
    pnh_.setParam("gate_count", static_cast<int>(msg.poses.size()));
    pnh_.setParam("trajectory_start_time", start_time_.toSec());
    ROS_INFO("Generated straight-line trajectory through %zu gates; duration %.3f s.",
             msg.poses.size(), execution_duration_);
  }

  // Part 1.3: sample simulation elapsed time and hold the terminal state.
  void publishDesiredState(const ros::TimerEvent&) {
    if (trajectory_.empty()) return;
    using namespace mav_trajectory_generation::derivative_order;
    const ros::Time now = ros::Time::now();
    const double duration = trajectory_.getMaxTime();

    const double elapsed=std::max(0.0,(now-start_time_).toSec());
    double t,h1,h2,h3,h4;
    pathClock(elapsed,t,h1,h2,h3,h4);
    if (t>=duration) { t=duration; h1=h2=h3=h4=0.0; }
    t=std::max(0.0,t);

    const Eigen::Vector3d p = trajectory_.evaluate(t, POSITION);

    const Eigen::Vector3d raw_v=trajectory_.evaluate(t,VELOCITY);
    const Eigen::Vector3d raw_a=trajectory_.evaluate(t,ACCELERATION);
    const Eigen::Vector3d v=raw_v*h1;
    const Eigen::Vector3d a=raw_a*h1*h1+raw_v*h2;

    const double yaw = yaw_trajectory_.evaluate(t, ORIENTATION)[0];
    trajectory_msgs::MultiDOFJointTrajectoryPoint next;
    next.time_from_start = ros::Duration(std::min(elapsed,execution_duration_));
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
    next.velocities[0].angular.z = yaw_trajectory_.evaluate(t, VELOCITY)[0]*h1;
    next.accelerations[0].linear.x = a.x();
    next.accelerations[0].linear.y = a.y();
    next.accelerations[0].linear.z = a.z();
    next.accelerations[0].angular.z = yaw_trajectory_.evaluate(t, ACCELERATION)[0]*h1*h1
        +yaw_trajectory_.evaluate(t, VELOCITY)[0]*h2;

    // ATTITUDE_FF_REFERENCE_V1: companion trajectory point; angular quantities
    // are expressed in its desired body frame. No position command is changed.
    // Fade in preview after the start maneuver. Keep the message timestamp
    // aligned with the current tracking target for the controller's freshness check.
    const double u = std::max(0.0, std::min(1.0, (elapsed - 3.0) / 0.5));
    const double preview = attitude_preview_ * u*u*(3.0-2.0*u);
    double ft=t, f1=h1, f2=h2, f3=h3, f4=h4;
    if (preview > 0.0) {
      pathClock(elapsed+preview, ft, f1, f2, f3, f4);
      if (ft>=duration) { ft=duration; f1=f2=f3=f4=0.0; }
    }
    const Eigen::Vector3d fv = trajectory_.evaluate(ft, VELOCITY);
    const Eigen::Vector3d fa = trajectory_.evaluate(ft, ACCELERATION);
    const Eigen::Vector3d fj = trajectory_.evaluate(ft, JERK);
    const Eigen::Vector3d fs = trajectory_.evaluate(ft, SNAP);
    const Eigen::Vector3d ff_accel = fa*f1*f1+fv*f2;
    const Eigen::Vector3d ff_jerk = fj*f1*f1*f1+3.0*fa*f1*f2+fv*f3;
    const Eigen::Vector3d ff_snap = fs*f1*f1*f1*f1
        +6.0*fj*f1*f1*f2+3.0*fa*f2*f2+4.0*fa*f1*f3+fv*f4;
    const double ff_yaw = yaw_trajectory_.evaluate(ft, ORIENTATION)[0];
    const double yaw_rate = yaw_trajectory_.evaluate(ft, VELOCITY)[0];
    const double ff_yaw_accel = yaw_trajectory_.evaluate(ft, ACCELERATION)[0]*f1*f1+yaw_rate*f2;
    Eigen::Matrix3d ff_R;
    Eigen::Vector3d ff_w, ff_alpha;
    if (lab4ff::compute(ff_accel, ff_jerk, ff_snap,
          ff_yaw, yaw_rate*f1, ff_yaw_accel, &ff_R, &ff_w, &ff_alpha)) {
      trajectory_msgs::MultiDOFJointTrajectoryPoint ff;
      ff.time_from_start=next.time_from_start;
      ff.transforms.resize(1); ff.velocities.resize(1); ff.accelerations.resize(1);
      const Eigen::Quaterniond fq(ff_R);
      ff.transforms[0].rotation.x=fq.x(); ff.transforms[0].rotation.y=fq.y();
      ff.transforms[0].rotation.z=fq.z(); ff.transforms[0].rotation.w=fq.w();
      ff.velocities[0].angular.x=ff_w.x(); ff.velocities[0].angular.y=ff_w.y(); ff.velocities[0].angular.z=ff_w.z();
      ff.accelerations[0].angular.x=ff_alpha.x(); ff.accelerations[0].angular.y=ff_alpha.y(); ff.accelerations[0].angular.z=ff_alpha.z();
      attitude_ff_pub_.publish(ff);
    }
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

    pnh_.param("path_speedup", path_speedup_, 1.0);
    pnh_.param("path_ramp", path_ramp_, 1.0);
    pnh_.param("attitude_preview", attitude_preview_, 0.0);
    if (!std::isfinite(attitude_preview_) || attitude_preview_ < 0.0 || attitude_preview_ > 0.08)
      throw std::invalid_argument("attitude_preview must be in [0, 0.08] seconds");
    ROS_INFO("ATTITUDE PREVIEW: %.3f s", attitude_preview_);
    pnh_.param("late_speedup", late_speedup_, 0.0);
    pnh_.param("late_after_gate", late_after_gate_, 8);
    pnh_.param("late_until_gate", late_until_gate_, 0);
    pnh_.param("turn_slowdown", turn_slowdown_, 0.0);
    pnh_.param("turn_after_gate", turn_after_gate_, 4);
    pnh_.param("turn_until_gate", turn_until_gate_, 7);
    if (!std::isfinite(turn_slowdown_) || turn_slowdown_ < 0.0 || turn_slowdown_ > 0.15 ||
        (turn_slowdown_ > 0.0 && (turn_after_gate_ < 3 || turn_until_gate_ <= turn_after_gate_)))
      throw std::invalid_argument("Invalid turn slowdown settings");
    pnh_.param("late_return_ramp", late_return_ramp_, 0.5);
    pnh_.param("late_final_rate", late_final_rate_, 0.0);
    pnh_.param("gate_window_retime", gate_window_retime_, false);
    if (!std::isfinite(late_final_rate_) || late_final_rate_ < 0.0 ||
        (late_final_rate_ > 0.0 && (late_until_gate_ <= 0 || late_speedup_ <= 0.0 ||
         late_final_rate_ < path_speedup_ || late_final_rate_ > path_speedup_ + late_speedup_)))
      throw std::invalid_argument("Late final rate requires an active return gate and must be within the base and boosted rates");
    if (late_until_gate_ < 0 || !std::isfinite(late_return_ramp_) || late_return_ramp_ < 0.5 ||
        late_return_ramp_ > 3.0)
      throw std::invalid_argument("Invalid late speed return settings");
    if (!std::isfinite(late_speedup_) || late_speedup_ < 0.0 ||
        (late_speedup_ > 0.0 && (late_after_gate_ < 3 || path_speedup_ + late_speedup_ > 1.25)))
      throw std::invalid_argument("Invalid late speedup or start gate");

    if (!std::isfinite(path_speedup_) || path_speedup_<1.0 || path_speedup_>1.25 ||
        !std::isfinite(path_ramp_) || path_ramp_<0.5 || path_ramp_>3.0)
      throw std::invalid_argument("path_speedup must be in [1,1.25]; path_ramp in [0.5,3]");

    if (!(v_max_ > 0 && a_max_ > 0 && time_scale_ > 0))
      throw std::invalid_argument("v_max, a_max, time_scale must be positive");
    state_sub_ = nh.subscribe("/current_state", 1, &WaypointFollower::onCurrentState, this);
    vertices_sub_ = nh.subscribe("/desired_traj_vertices", 1,
                               &WaypointFollower::generateOptimizedTrajectory, this);
    desired_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 1);
    attitude_ff_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_attitude_ff", 1);
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
