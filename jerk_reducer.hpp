#ifndef JERK_REDUCER__JERK_REDUCER_HPP_
#define JERK_REDUCER__JERK_REDUCER_HPP_

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <jerk_reducer/JerkReducerConfig.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace jerk_reducer
{

class JerkReducer
{
public:
  JerkReducer();
  ~JerkReducer();

  void spin();

private:
  double clamp(double x, double lo, double hi) const;
  double sgn(double x) const;

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr & msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr & msg);
  void dynamicReconfigureCallback(
    jerk_reducer::JerkReducerConfig & config, uint32_t level);

  void limitBodyJerkAndPreserveDirection(
    double cv_x, double cv_y, double cv_w,
    double cmd_x, double cmd_y, double cmd_w,
    double dt,
    double & out_x, double & out_y, double & out_w,
    geometry_msgs::Twist & raw_jerk_msg,
    geometry_msgs::Twist & limited_jerk_msg);

  void filterTimerCallback(const ros::TimerEvent &);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher filtered_cmd_pub_;
  ros::Publisher raw_jerk_pub_;
  ros::Publisher limited_jerk_pub_;
  ros::Timer filter_timer_;

  boost::shared_ptr<dynamic_reconfigure::Server<jerk_reducer::JerkReducerConfig>> dr_server_;

  std::mutex mutex_;

  geometry_msgs::Twist command_;
  geometry_msgs::Twist current_vel_;
  geometry_msgs::Twist last_cmd_;

  bool received_first_command_;
  bool stopped_;
  bool open_loop_;
  bool scale_velocities_;
  bool jerk_state_initialized_;

  std::vector<double> max_velocities_;
  std::vector<double> min_velocities_;
  std::vector<double> max_accels_;
  std::vector<double> max_decels_;
  std::vector<double> max_jerks_;
  std::vector<double> deadband_velocities_;
  std::vector<double> prev_accel_;

  double filter_frequency_;
  double velocity_timeout_;
  std::string odom_topic_;

  int max_jerk_iterations_;

  double v_prev_;
  double omega_prev_;
  double vdot_prev_;
  double omegadot_prev_;

  ros::Time last_command_time_;
};

}  // namespace jerk_reducer

#endif  // JERK_REDUCER__JERK_REDUCER_HPP_