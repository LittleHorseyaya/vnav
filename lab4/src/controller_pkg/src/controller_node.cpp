#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>

#define PI M_PI

#include <eigen3/Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>

class controllerNode{
  ros::NodeHandle nh;

  ros::Subscriber des_state_sub;
  ros::Subscriber cur_state_sub;
  ros::Publisher rotor_speed_pub;
  ros::Timer control_timer;

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
      ROS_INFO("Gain values:\nkx: %f \nkv: %f \nkr: %f \nkomega: %f\n", kx, kv, kr, komega);

      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;

      double arm = d / sqrt(2.0);
      F2W << cf,          cf,          cf,          cf,
             arm*cf,      arm*cf,     -arm*cf,     -arm*cf,
            -arm*cf,      arm*cf,      arm*cf,     -arm*cf,
             cd,         -cd,          cd,         -cd;
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
