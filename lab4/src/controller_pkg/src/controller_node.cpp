#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <eigen3/Eigen/Cholesky>
#include <algorithm>
#include <eigen3/Eigen/Geometry>

#define PI M_PI

#include <eigen3/Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>

class controllerNode{
  ros::NodeHandle nh;

  ros::Subscriber des_state_sub;
  ros::Subscriber cur_state_sub;
  ros::Publisher rotor_speed_pub;
  ros::Timer control_timer;


  // ATTITUDE_FF_EXPERIMENT_V1: default gain zero preserves the original torque.
  ros::Subscriber attitude_ff_sub_;
  Eigen::Matrix3d ff_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d ff_omega_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ff_alpha_ = Eigen::Vector3d::Zero();
  ros::Time ff_received_;
  bool have_ff_ = false;
  double ff_phase_ = 0.0, desired_phase_ = 0.0;
  double ff_gain_ = 0.0, ff_start_ = 3.5, ff_ramp_ = 0.5, ff_max_torque_ = 2.0;


  // LEARNING_FF_EXPERIMENT_V1
  struct LearningRow { double phase; Eigen::Vector3d acceleration; };
  std::vector<LearningRow> learning_rows_;
  bool learning_loaded_=false, learning_phase_ready_=false, learning_enabled_=false;
  double learning_gain_=0.0, learning_limit_=0.6, learning_geometry_=0.0;
  double learning_rate_=1.0, learning_ramp_=1.0, learning_prefix_=0.0;
  int learning_gate_count_=0;

  double kx, kv, kr, komega;
  double kx_z, kv_z;
  double m, g, d, cf, cd;
  Eigen::Matrix3d J;
  Eigen::Vector3d e3;
  Eigen::MatrixXd F2W;

  Eigen::Vector3d x, v, omega;
  Eigen::Matrix3d R;
  Eigen::Vector3d xd, vd, ad;
  double yawd;
  double hz;

  bool have_current_state;
  bool have_desired_state;

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(100.0),
    have_current_state(false),have_desired_state(false){

      des_state_sub = nh.subscribe("desired_state", 1, &controllerNode::onDesiredState, this);
      cur_state_sub = nh.subscribe("current_state", 1, &controllerNode::onCurrentState, this);
      rotor_speed_pub = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
      control_timer = nh.createTimer(ros::Duration(1.0/hz), &controllerNode::controlLoop, this);

      nh.param("kx", kx, 1.0);
      nh.param("kv", kv, 2.0);
      nh.param("kr", kr, 1.0);
      nh.param("komega", komega, 0.5);
      nh.param("kx_z", kx_z, 4.0);
      nh.param("kv_z", kv_z, 4.0);
      ros::NodeHandle ff_pnh("~");
      ff_pnh.param("attitude_ff_gain", ff_gain_, 0.0);
      ff_pnh.param("attitude_ff_start", ff_start_, 3.5);
      ff_pnh.param("attitude_ff_ramp", ff_ramp_, 0.5);
      ff_pnh.param("attitude_ff_max_torque", ff_max_torque_, 2.0);
      double learning_gain_requested=0.0;
      std::string learning_file;
      ff_pnh.param("learning_gain",learning_gain_requested,0.0);
      ff_pnh.param("learning_max_accel",learning_limit_,0.6);
      ff_pnh.param("learning_file",learning_file,std::string(""));
      if (std::isfinite(learning_gain_requested) && learning_gain_requested>=0.0 &&
          learning_gain_requested<=1.0 && std::isfinite(learning_limit_) &&
          learning_limit_>0.0 && learning_limit_<=1.5) {
        learning_gain_=learning_gain_requested;
        if (learning_gain_>0.0) learning_loaded_=loadLearningTable(learning_file);
      } else {
        ROS_ERROR("Invalid learning parameters: disabling learning feedforward");
      }
      ff_pnh.setParam("learning_table_loaded",learning_loaded_);
      ff_pnh.setParam("learning_active",false);

      if (!std::isfinite(ff_gain_) || ff_gain_ < 0 || ff_gain_ > 1 ||
          !std::isfinite(ff_start_) || ff_start_ < 0 ||
          !std::isfinite(ff_ramp_) || ff_ramp_ <= 0 ||
          !std::isfinite(ff_max_torque_) || ff_max_torque_ <= 0) {
        ROS_ERROR("Invalid attitude FF parameters: disabling feedforward");
        ff_gain_ = 0.0;
      }
      attitude_ff_sub_ = nh.subscribe("/desired_attitude_ff", 1,
                                     &controllerNode::onAttitudeFeedforward, this);

      ROS_INFO("Gain values:\nkx: %f \nkv: %f \nkr: %f \nkomega: %f\n", kx, kv, kr, komega);

      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;

      // IDENTIFIED_INERTIA_EXPERIMENT_V1
      // Effective inertia under the existing rotor-to-wrench allocation.
      // The fit is not a certified measurement of the simulator's internal tensor.
      J = Eigen::Matrix3d::Identity();
      bool use_identified_inertia = false;
      ff_pnh.param("use_identified_inertia", use_identified_inertia, false);
      if (use_identified_inertia) {
        double jxx, jyy, jzz, jxy, jxz, jyz;
        ff_pnh.param("inertia_xx", jxx, 0.102380);
        ff_pnh.param("inertia_yy", jyy, 0.189959);
        ff_pnh.param("inertia_zz", jzz, 0.092444);
        ff_pnh.param("inertia_xy", jxy, 0.001612);
        ff_pnh.param("inertia_xz", jxz, -0.003000);
        ff_pnh.param("inertia_yz", jyz, -0.004245);
        J << jxx,jxy,jxz, jxy,jyy,jyz, jxz,jyz,jzz;
        bool valid=J.allFinite();
        if (valid) {
          const Eigen::LLT<Eigen::Matrix3d> factor(J);
          valid=(factor.info()==Eigen::Success);
        }
        if (!valid) {
          ROS_ERROR("Invalid identified inertia; restoring unit inertia");
          J=Eigen::Matrix3d::Identity();
          use_identified_inertia=false;
        }
      }
      ff_pnh.setParam("inertia_model_active",use_identified_inertia);
      ROS_INFO("INERTIA MODEL: identified=%s, diagonal=[%.6f, %.6f, %.6f]",
               use_identified_inertia ? "true" : "false",J(0,0),J(1,1),J(2,2));


      double arm = d / sqrt(2.0);
      F2W << cf,          cf,          cf,          cf,
             arm*cf,      arm*cf,     -arm*cf,     -arm*cf,
            -arm*cf,      arm*cf,      arm*cf,     -arm*cf,
             cd,         -cd,          cd,         -cd;
  }



  bool loadLearningTable(const std::string& filename) {
    std::ifstream input(filename.c_str());
    std::string version,line;
    if (!(input>>version>>learning_geometry_>>learning_gate_count_) ||
        version!="LAB4_LEARNING_V1" || !std::isfinite(learning_geometry_) ||
        learning_geometry_<=0.0 || learning_gate_count_<1) {
      ROS_ERROR("Cannot read compatible learning table: %s",filename.c_str());
      return false;
    }
    std::getline(input,line);
    std::vector<LearningRow> rows;
    while (std::getline(input,line)) {
      if (line.empty()) continue;
      std::replace(line.begin(),line.end(),',',' ');
      std::istringstream parser(line);
      LearningRow row;
      if (!(parser>>row.phase>>row.acceleration.x()>>row.acceleration.y()>>row.acceleration.z()) ||
          !std::isfinite(row.phase) || row.phase<0.0 || !row.acceleration.allFinite() ||
          row.acceleration.norm()>5.0 || std::abs(row.acceleration.z())>1e-9 ||
          (!rows.empty() && row.phase<=rows.back().phase)) {
        ROS_ERROR("Invalid learning table row; learning disabled");
        return false;
      }
      rows.push_back(row);
    }
    if (rows.size()<2 || rows.front().phase>1e-6 ||
        std::abs(rows.back().phase-learning_geometry_)>1e-3) {
      ROS_ERROR("Learning table does not cover the reference trajectory");
      return false;
    }
    learning_rows_.swap(rows);
    ROS_INFO("LEARNING TABLE: loaded %zu phase samples",learning_rows_.size());
    return true;
  }

  void initializeLearningPhase() {
    learning_phase_ready_=true;
    double geometry=0.0;
    int gates=0;
    const std::string base="/trajectory_generation_node/";
    bool valid=nh.getParam(base+"path_speedup",learning_rate_)
        && nh.getParam(base+"path_ramp",learning_ramp_)
        && nh.getParam(base+"path_clock_prefix",learning_prefix_)
        && nh.getParam(base+"geometric_duration",geometry)
        && nh.getParam(base+"gate_count",gates);
    valid=valid && std::isfinite(learning_rate_) && learning_rate_>=1.0 && learning_rate_<=1.25
        && std::isfinite(learning_ramp_) && learning_ramp_>=0.5 && learning_ramp_<=3.0
        && std::isfinite(learning_prefix_) && learning_prefix_>=0.0
        && std::isfinite(geometry) && std::abs(geometry-learning_geometry_)<0.01
        && gates==learning_gate_count_;
    learning_enabled_=valid;
    ros::NodeHandle("~").setParam("learning_active",valid);
    if (valid) ROS_INFO("LEARNING ACTIVE: gain=%.3f, limit=%.3f m/s^2",learning_gain_,learning_limit_);
    else ROS_ERROR("Learning disabled: phase metadata or reference geometry mismatch");
  }

  Eigen::Vector3d learningCorrection() const {
    if (!learning_enabled_ || learning_rows_.size()<2 || desired_phase_<=3.5)
      return Eigen::Vector3d::Zero();
    const double e=desired_phase_;
    double phase=e;
    const double x=(e-learning_prefix_)/learning_ramp_;
    if (x>=1.0) phase+=(learning_rate_-1.0)*(e-learning_prefix_-0.5*learning_ramp_);
    else if (x>0.0) {
      const double x2=x*x,x3=x2*x,x4=x3*x,x5=x4*x,x6=x5*x,x7=x6*x,x8=x7*x;
      phase+=(learning_rate_-1.0)*learning_ramp_*(7*x5-14*x6+10*x7-2.5*x8);
    }
    if (!std::isfinite(phase) || phase<learning_rows_.front().phase || phase>learning_rows_.back().phase)
      return Eigen::Vector3d::Zero();
    auto high=std::lower_bound(learning_rows_.begin(),learning_rows_.end(),phase,
        [](const LearningRow& row,double value){return row.phase<value;});
    if (high==learning_rows_.begin()) return Eigen::Vector3d::Zero();
    if (high==learning_rows_.end()) return Eigen::Vector3d::Zero();
    const auto low=high-1;
    const double u=(phase-low->phase)/(high->phase-low->phase);
    Eigen::Vector3d correction=(1-u)*low->acceleration+u*high->acceleration;
    const double blend=std::max(0.0,std::min(1.0,(e-3.5)/0.5));
    correction*=learning_gain_*blend*blend*(3-2*blend);
    const double magnitude=correction.norm();
    if (!correction.allFinite() || !std::isfinite(magnitude)) return Eigen::Vector3d::Zero();
    if (magnitude>learning_limit_) correction*=learning_limit_/magnitude;
    return correction;
  }

  void onAttitudeFeedforward(const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg) {
    have_ff_ = false;
    if (msg.transforms.size()!=1 || msg.velocities.size()!=1 || msg.accelerations.size()!=1) return;
    const auto& q = msg.transforms[0].rotation;
    Eigen::Quaterniond eq(q.w,q.x,q.y,q.z);
    if (!eq.coeffs().allFinite() || eq.norm()<1e-6) return;
    eq.normalize();
    const auto& w=msg.velocities[0].angular;
    const auto& a=msg.accelerations[0].angular;
    ff_omega_ << w.x,w.y,w.z;
    ff_alpha_ << a.x,a.y,a.z;
    ff_R_ = eq.toRotationMatrix();
    ff_phase_ = msg.time_from_start.toSec();
    if (!std::isfinite(ff_phase_) || !ff_omega_.allFinite() || !ff_alpha_.allFinite()) return;
    ff_received_ = ros::Time::now();
    have_ff_ = true;
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){
      if (des_state.transforms.empty() ||
          des_state.velocities.empty() ||
          des_state.accelerations.empty()) {
          ROS_WARN_THROTTLE(1.0, "Desired state message is incomplete.");
          return;
      }

      xd << des_state.transforms[0].translation.x,
            des_state.transforms[0].translation.y,
            des_state.transforms[0].translation.z;

      vd << des_state.velocities[0].linear.x,
            des_state.velocities[0].linear.y,
            des_state.velocities[0].linear.z;

      ad << des_state.accelerations[0].linear.x,
            des_state.accelerations[0].linear.y,
            des_state.accelerations[0].linear.z;

      tf2::Quaternion q_des;
      tf2::fromMsg(des_state.transforms[0].rotation, q_des);
      yawd = tf2::getYaw(q_des);

      desired_phase_ = des_state.time_from_start.toSec();
      if (learning_loaded_ && !learning_phase_ready_) initializeLearningPhase();
      have_desired_state = true;
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      x << cur_state.pose.pose.position.x,
           cur_state.pose.pose.position.y,
           cur_state.pose.pose.position.z;

      v << cur_state.twist.twist.linear.x,
           cur_state.twist.twist.linear.y,
           cur_state.twist.twist.linear.z;

      tf2::Quaternion q_cur;
      tf2::fromMsg(cur_state.pose.pose.orientation, q_cur);
      tf2::Matrix3x3 R_tf(q_cur);
      R << R_tf[0][0], R_tf[0][1], R_tf[0][2],
           R_tf[1][0], R_tf[1][1], R_tf[1][2],
           R_tf[2][0], R_tf[2][1], R_tf[2][2];

      Eigen::Vector3d omega_world;
      omega_world << cur_state.twist.twist.angular.x,
                     cur_state.twist.twist.angular.y,
                     cur_state.twist.twist.angular.z;
      omega = R.transpose() * omega_world;

      have_current_state = true;
  }

  void controlLoop(const ros::TimerEvent& t){
    if (!have_current_state || !have_desired_state) {
        ROS_WARN_THROTTLE(2.0, "Waiting for current_state and desired_state.");
        return;
    }

    Eigen::Vector3d ex, ev, er, eomega;

    ex = x - xd;
    ev = v - vd;

    Eigen::Vector3d f_des;
    f_des.x() = m * (ad.x() - kx * ex.x() - kv * ev.x());
    f_des.y() = m * (ad.y() - kx * ex.y() - kv * ev.y());
    f_des.z() = m * (ad.z() + g - kx_z * ex.z() - kv_z * ev.z());

    f_des += m*learningCorrection();

    double f_norm = f_des.norm();
    if (f_norm < 1e-6) return;

    Eigen::Vector3d b3d = f_des / f_norm;

    Eigen::Vector3d xC;
    xC << cos(yawd), sin(yawd), 0;
    Eigen::Vector3d b2d = b3d.cross(xC);
    if (b2d.squaredNorm() < 1e-12) {
        xC = Eigen::Vector3d(-sin(yawd), cos(yawd), 0);
        b2d = b3d.cross(xC);
    }
    b2d = b2d.normalized();

    Eigen::Vector3d b1d = b2d.cross(b3d);
    b1d = b1d.normalized();

    Eigen::Matrix3d Rd;
    Rd.col(0) = b1d;
    Rd.col(1) = b2d;
    Rd.col(2) = b3d;

    er = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd);
    eomega = omega;

    Eigen::Vector3d tau = -kr * er - komega * eomega;
    // Add bounded analytic reference-rate/acceleration feedforward, with a
    // smooth delayed activation. Existing kx/kv/kr/komega and feedback stay intact.
    if (ff_gain_ > 0.0 && have_ff_ && desired_phase_ > ff_start_) {
      const double age=(ros::Time::now()-ff_received_).toSec();
      if (age>=0.0 && age<=0.06 && std::abs(ff_phase_-desired_phase_)<=0.04) {
        const double u=std::max(0.0,std::min(1.0,(desired_phase_-ff_start_)/ff_ramp_));
        const double beta=ff_gain_*u*u*(3.0-2.0*u);
        const double dbeta=ff_gain_*6.0*u*(1.0-u)/ff_ramp_;
        const Eigen::Matrix3d map=R.transpose()*ff_R_;
        const Eigen::Vector3d mapped_rate=map*ff_omega_;
        Eigen::Vector3d extra=komega*beta*mapped_rate
            + J*(beta*map*ff_alpha_ + dbeta*mapped_rate
                 - beta*omega.cross(mapped_rate))
            + beta*omega.cross(J*omega);
        const double extra_norm=extra.norm();
        if (extra.allFinite() && std::isfinite(extra_norm)) {
          if (extra_norm>ff_max_torque_) extra*=ff_max_torque_/extra_norm;
          tau+=extra;
        }
      }
    }


    double thrust = f_des.dot(R * e3);

    Eigen::Vector4d wrench;
    wrench << thrust, tau;

    Eigen::Vector4d w_sq = F2W.inverse() * wrench;
    ROS_INFO_THROTTLE(0.5, "w_sq: [%.2f, %.2f, %.2f, %.2f]", 
        w_sq(0), w_sq(1), w_sq(2), w_sq(3));

    Eigen::Vector4d w;
    for (int i = 0; i < 4; i++) {
        w(i) = signed_sqrt(w_sq(i));
    }

    mav_msgs::Actuators cmd;
    cmd.angular_velocities.clear();
    for (int i = 0; i < 4; i++) {
        cmd.angular_velocities.push_back(w(i));
    }
    rotor_speed_pub.publish(cmd);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  controllerNode n;
  ros::spin();
}
