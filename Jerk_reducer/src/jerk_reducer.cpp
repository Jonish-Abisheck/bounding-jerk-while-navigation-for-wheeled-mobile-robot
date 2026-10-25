// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include "jerk_reducer/jerk_reducer.hpp"

#include <algorithm>
#include <cmath>

namespace jerk_reducer
{

JerkReducer::JerkReducer()
: pnh_("~"),
  received_first_command_(false),
  stopped_(true),
  open_loop_(true),
  scale_velocities_(false),
  prev_accel_(3, 0.0),
  v_prev_(0.0),
  omega_prev_(0.0),
  vdot_prev_(0.0),
  omegadot_prev_(0.0),
  jerk_state_initialized_(false)
{
  pnh_.param("filter_frequency", filter_frequency_, 20.0);

  std::string feedback;
  pnh_.param<std::string>("feedback", feedback, "OPEN_LOOP");
  if (feedback == "OPEN_LOOP") {
    open_loop_ = true;
  } else if (feedback == "CLOSED_LOOP") {
    open_loop_ = false;
  } else {
    ROS_ERROR("[JerkReducer] Invalid feedback '%s'. Defaulting to OPEN_LOOP.", feedback.c_str());
    open_loop_ = true;
  }

  pnh_.param("scale_velocities", scale_velocities_, false);
  pnh_.param("velocity_timeout", velocity_timeout_, 1.0);
  pnh_.param<std::string>("odom_topic", odom_topic_, "odom");

  std::vector<double> def_max_vel  = {1.0,  0.0,  3.5};
  std::vector<double> def_min_vel  = {-1.0, 0.0, -3.5};
  std::vector<double> def_max_acc  = {2.5,  0.0,  3.2};
  std::vector<double> def_max_dec  = {-2.5, 0.0, -3.2};
  std::vector<double> def_max_jerk = {10.0, 5.0, 15.0};
  std::vector<double> def_deadband = {0.0,  0.0,  0.0};

  pnh_.param("max_velocity",      max_velocities_,      def_max_vel);
  pnh_.param("min_velocity",      min_velocities_,      def_min_vel);
  pnh_.param("max_accel",         max_accels_,          def_max_acc);
  pnh_.param("max_decel",         max_decels_,          def_max_dec);
  pnh_.param("max_jerk",          max_jerks_,           def_max_jerk);
  pnh_.param("deadband_velocity", deadband_velocities_, def_deadband);

  if (max_velocities_.size() != 3 || min_velocities_.size() != 3 ||
      max_accels_.size() != 3     || max_decels_.size() != 3     ||
      max_jerks_.size() != 3      || deadband_velocities_.size() != 3)
  {
    ROS_FATAL("[JerkReducer] All limit vectors must have size 3 [vx, vy, wz]!");
    ros::shutdown();
    return;
  }

  for (int i = 0; i < 3; ++i) {
    if (max_decels_[i] > 0.0) {
      ROS_FATAL("[JerkReducer] max_decel[%d]=%.3f must be <= 0!", i, max_decels_[i]);
      ros::shutdown();
      return;
    }
    if (max_accels_[i] < 0.0) {
      ROS_FATAL("[JerkReducer] max_accel[%d]=%.3f must be >= 0!", i, max_accels_[i]);
      ros::shutdown();
      return;
    }
    if (max_jerks_[i] < 0.0) {
      ROS_FATAL("[JerkReducer] max_jerk[%d]=%.3f must be >= 0!", i, max_jerks_[i]);
      ros::shutdown();
      return;
    }
  }

  dr_server_ = boost::make_shared<
    dynamic_reconfigure::Server<jerk_reducer::JerkReducerConfig>>(pnh_);

  jerk_reducer::JerkReducerConfig init_config;
  init_config.filter_frequency = filter_frequency_;
  init_config.velocity_timeout = velocity_timeout_;

  init_config.max_vel_x        = max_velocities_[0];
  init_config.min_vel_x        = min_velocities_[0];
  init_config.max_vel_y        = max_velocities_[1];
  init_config.min_vel_y        = min_velocities_[1];
  init_config.max_vel_theta    = max_velocities_[2];
  init_config.min_vel_theta    = min_velocities_[2];

  init_config.max_accel_x      = max_accels_[0];
  init_config.max_accel_y      = max_accels_[1];
  init_config.max_accel_theta  = max_accels_[2];

  init_config.max_decel_x      = max_decels_[0];
  init_config.max_decel_y      = max_decels_[1];
  init_config.max_decel_theta  = max_decels_[2];

  init_config.max_jerk_x       = max_jerks_[0];
  init_config.max_jerk_y       = max_jerks_[1];
  init_config.max_jerk_theta   = max_jerks_[2];

  dr_server_->updateConfig(init_config);
  dr_server_->setCallback(
    boost::bind(&JerkReducer::dynamicReconfigureCallback, this, _1, _2));

  cmd_sub_          = nh_.subscribe("cmd_vel", 1, &JerkReducer::cmdVelCallback, this);
  filtered_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_filtered", 1);
  raw_jerk_pub_     = nh_.advertise<geometry_msgs::Twist>("cmd_jerk_raw", 1);
  limited_jerk_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_jerk_limited", 1);

  if (!open_loop_) {
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &JerkReducer::odomCallback, this);
  }

  double dt = 1.0 / filter_frequency_;
  filter_timer_ = nh_.createTimer(
    ros::Duration(dt), &JerkReducer::filterTimerCallback, this);

  last_command_time_ = ros::Time(0);

  ROS_INFO("[JerkReducer] Ready. freq=%.1f Hz, mode=%s",
           filter_frequency_, open_loop_ ? "OPEN_LOOP" : "CLOSED_LOOP");
  ROS_INFO("[JerkReducer] max_velocity=[%.2f, %.2f, %.2f]",
           max_velocities_[0], max_velocities_[1], max_velocities_[2]);
  ROS_INFO("[JerkReducer] max_accel   =[%.2f, %.2f, %.2f]",
           max_accels_[0], max_accels_[1], max_accels_[2]);
  ROS_INFO("[JerkReducer] max_jerk    =[%.2f, %.2f, %.2f]",
           max_jerks_[0], max_jerks_[1], max_jerks_[2]);
}

JerkReducer::~JerkReducer()
{
  filter_timer_.stop();
}

void JerkReducer::spin()
{
  ros::spin();
}

double JerkReducer::clamp(double x, double lo, double hi) const
{
  return std::max(lo, std::min(x, hi));
}

double JerkReducer::sgn(double x) const
{
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return 0.0;
}

void JerkReducer::cmdVelCallback(const geometry_msgs::Twist::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto ok = [](double v) { return std::isfinite(v); };
  if (!ok(msg->linear.x) || !ok(msg->linear.y) || !ok(msg->linear.z) ||
      !ok(msg->angular.x) || !ok(msg->angular.y) || !ok(msg->angular.z))
  {
    ROS_WARN_THROTTLE(1.0, "[JerkReducer] NaN/Inf in cmd_vel, ignoring.");
    return;
  }

  command_ = *msg;
  last_command_time_ = ros::Time::now();
  received_first_command_ = true;
}

void JerkReducer::odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_vel_ = msg->twist.twist;
}

void JerkReducer::dynamicReconfigureCallback(
  jerk_reducer::JerkReducerConfig & config, uint32_t)
{
  std::lock_guard<std::mutex> lock(mutex_);

  filter_frequency_   = config.filter_frequency;
  velocity_timeout_   = config.velocity_timeout;

  max_velocities_[0]  = config.max_vel_x;
  max_velocities_[1]  = config.max_vel_y;
  max_velocities_[2]  = config.max_vel_theta;

  min_velocities_[0]  = config.min_vel_x;
  min_velocities_[1]  = config.min_vel_y;
  min_velocities_[2]  = config.min_vel_theta;

  max_accels_[0]      = config.max_accel_x;
  max_accels_[1]      = config.max_accel_y;
  max_accels_[2]      = config.max_accel_theta;

  max_decels_[0]      = config.max_decel_x;
  max_decels_[1]      = config.max_decel_y;
  max_decels_[2]      = config.max_decel_theta;

  max_jerks_[0]       = config.max_jerk_x;
  max_jerks_[1]       = config.max_jerk_y;
  max_jerks_[2]       = config.max_jerk_theta;

  filter_timer_.stop();
  filter_timer_ = nh_.createTimer(
    ros::Duration(1.0 / filter_frequency_),
    &JerkReducer::filterTimerCallback, this);

  ROS_INFO("[JerkReducer] Reconfigured: max_vel=[%.2f, %.2f, %.2f], max_jerk=[%.2f, %.2f, %.2f], freq=%.1f Hz",
           max_velocities_[0], max_velocities_[1], max_velocities_[2],
           max_jerks_[0], max_jerks_[1], max_jerks_[2],
           filter_frequency_);
}

void JerkReducer::limitBodyJerkAndPreserveDirection(
  double cv_x, double cv_y, double cv_w,
  double cmd_x, double cmd_y, double cmd_w,
  double dt,
  double & out_x, double & out_y, double & out_w,
  geometry_msgs::Twist & raw_jerk_msg,
  geometry_msgs::Twist & limited_jerk_msg)
{
  const double eps = 1e-6;

  if (!jerk_state_initialized_) {
    v_prev_         = cv_x;
    omega_prev_     = cv_w;
    vdot_prev_      = 0.0;
    omegadot_prev_  = 0.0;
    jerk_state_initialized_ = true;
  }

  auto getAccelBounds = [&](double curr, double cmd,
                            double & accel_max, double & accel_min,
                            double max_accel, double max_decel) {
    if (std::fabs(cmd) >= std::fabs(curr) && curr * cmd >= 0.0) {
      accel_max =  max_accel;
      accel_min = -max_accel;
    } else {
      accel_max = -max_decel;
      accel_min =  max_decel;
    }
  };

  // UNCHANGED: used by the limiter and the eta check (previous-step terms).
  auto computeJerkForOutput = [&](double test_x, double test_w,
                                  double & j_tan, double & j_lat, double & j_ang,
                                  double & vdot,  double & omegadot) {
    vdot     = (test_x - v_prev_)     / dt;
    omegadot = (test_w - omega_prev_) / dt;
    j_tan    = (vdot     - vdot_prev_)     / dt - v_prev_ * omega_prev_ * omega_prev_;
    j_lat    = 2.0 * vdot * omega_prev_    + v_prev_ * omegadot;
    j_ang    = (omegadot - omegadot_prev_) / dt;
  };

  double vdot_cmd      = (cmd_x - v_prev_)       / dt;
  double vddot_cmd     = (vdot_cmd - vdot_prev_) / dt;
  double omegadot_cmd  = (cmd_w - omega_prev_)   / dt;
  double j_ang_raw     = (omegadot_cmd - omegadot_prev_) / dt;

  double j_tan_raw = vddot_cmd - v_prev_ * omega_prev_ * omega_prev_;
  double j_lat_raw = 2.0 * vdot_cmd * omega_prev_ + v_prev_ * omegadot_cmd;

  raw_jerk_msg.linear.x  = j_tan_raw;
  raw_jerk_msg.linear.y  = j_lat_raw;
  raw_jerk_msg.linear.z  = 0.0;
  raw_jerk_msg.angular.x = 0.0;
  raw_jerk_msg.angular.y = 0.0;
  raw_jerk_msg.angular.z = j_ang_raw;

  double accel_max_x, accel_min_x;
  double accel_max_w, accel_min_w;
  getAccelBounds(cv_x, cmd_x, accel_max_x, accel_min_x, max_accels_[0], max_decels_[0]);
  getAccelBounds(cv_w, cmd_w, accel_max_w, accel_min_w, max_accels_[2], max_decels_[2]);

  double j_tan_target  = clamp(j_tan_raw, -max_jerks_[0], max_jerks_[0]);
  double vddot_limited = j_tan_target + v_prev_ * omega_prev_ * omega_prev_;
  double vdot_limited  = vdot_prev_ + vddot_limited * dt;
  vdot_limited         = clamp(vdot_limited, accel_min_x, accel_max_x);
  double x_jerk        = v_prev_ + vdot_limited * dt;
  x_jerk               = clamp(x_jerk, min_velocities_[0], max_velocities_[0]);

  double j_lat_target = clamp(j_lat_raw, -max_jerks_[1], max_jerks_[1]);
  double w_jerk       = omega_prev_;
  if (std::fabs(v_prev_) > eps) {
    double omegadot_from_jlat =
      (j_lat_target - 2.0 * vdot_limited * omega_prev_) / v_prev_;
    omegadot_from_jlat = clamp(omegadot_from_jlat, accel_min_w, accel_max_w);
    w_jerk = omega_prev_ + omegadot_from_jlat * dt;
  } else {
    double omegadot_limited = clamp(omegadot_cmd, accel_min_w, accel_max_w);
    w_jerk = omega_prev_ + omegadot_limited * dt;
  }
  w_jerk = clamp(w_jerk, min_velocities_[2], max_velocities_[2]);

  double jerk_vdot = 0.0, jerk_omegadot = 0.0;
  double jerk_jtan = 0.0, jerk_jlat = 0.0, jerk_jang = 0.0;
  computeJerkForOutput(x_jerk, w_jerk,
                       jerk_jtan, jerk_jlat, jerk_jang,
                       jerk_vdot, jerk_omegadot);

  double final_x        = x_jerk;
  double final_y        = cv_y;
  double final_w        = w_jerk;
  double final_vdot     = jerk_vdot;
  double final_omegadot = jerk_omegadot;
  double final_jtan     = jerk_jtan;
  double final_jlat     = jerk_jlat;
  double final_jang     = jerk_jang;

  double eta    = 1.0;
  bool can_scale = false;

  auto updateEta = [&](double curr, double cmd, double lim) {
    double req   = cmd - curr;
    double allow = lim - curr;
    if (std::fabs(req) > eps) {
      double e = allow / req;
      eta       = std::min(eta, clamp(e, 0.0, 1.0));
      can_scale = true;
    }
  };

  updateEta(cv_x, cmd_x, x_jerk);
  updateEta(cv_w, cmd_w, w_jerk);

  if (can_scale) {
    double cand_x = cv_x + eta * (cmd_x - cv_x);
    double cand_y = cv_y + eta * (cmd_y - cv_y);
    double cand_w = cv_w + eta * (cmd_w - cv_w);

    cand_x = clamp(cand_x, min_velocities_[0], max_velocities_[0]);
    cand_y = clamp(cand_y, min_velocities_[1], max_velocities_[1]);
    cand_w = clamp(cand_w, min_velocities_[2], max_velocities_[2]);

    double cand_vdot = 0.0, cand_omegadot = 0.0;
    double cand_jtan = 0.0, cand_jlat = 0.0, cand_jang = 0.0;
    computeJerkForOutput(cand_x, cand_w,
                         cand_jtan, cand_jlat, cand_jang,
                         cand_vdot, cand_omegadot);

    bool jerk_ok =
      (max_jerks_[0] <= 0.0 || std::fabs(cand_jtan) <= max_jerks_[0]) &&
      (max_jerks_[1] <= 0.0 || std::fabs(cand_jlat) <= max_jerks_[1]);

    if (jerk_ok) {
      final_x        = cand_x;
      final_y        = cand_y;
      final_w        = cand_w;
      final_vdot     = cand_vdot;
      final_omegadot = cand_omegadot;
      final_jtan     = cand_jtan;
      final_jlat     = cand_jlat;
      final_jang     = cand_jang;
    }
  }

  out_x = final_x;
  out_y = final_y;
  out_w = final_w;

  // ---- NEW: recompute the jerk to publish using current-step (k) values ----
  // vdot_prev_ / omegadot_prev_ still hold step k-1 here (state updated below).
  const double pub_jtan =
    (final_vdot - vdot_prev_) / dt - final_x * final_w * final_w;
  const double pub_jlat =
    2.0 * final_vdot * final_w + final_x * final_omegadot;
  const double pub_jang =
    (final_omegadot - omegadot_prev_) / dt;

  const double pub_raw_jtan =
    (vdot_cmd - vdot_prev_) / dt - cmd_x * cmd_w * cmd_w;
  const double pub_raw_jlat =
    2.0 * vdot_cmd * cmd_w + cmd_x * omegadot_cmd;

  raw_jerk_msg.linear.x  = pub_raw_jtan;
  raw_jerk_msg.linear.y  = pub_raw_jlat;
  raw_jerk_msg.angular.z = j_ang_raw;

  limited_jerk_msg.linear.x  = pub_jtan;
  limited_jerk_msg.linear.y  = pub_jlat;
  limited_jerk_msg.linear.z  = 0.0;
  limited_jerk_msg.angular.x = 0.0;
  limited_jerk_msg.angular.y = 0.0;
  limited_jerk_msg.angular.z = pub_jang;
  // ---------------------------------------------------------------------------

  prev_accel_[0]  = final_vdot;
  prev_accel_[1]  = (out_y - cv_y) / dt;
  prev_accel_[2]  = final_omegadot;

  vdot_prev_     = final_vdot;
  omegadot_prev_ = final_omegadot;
  v_prev_        = out_x;
  omega_prev_    = out_w;
}

void JerkReducer::filterTimerCallback(const ros::TimerEvent &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!received_first_command_) {
    return;
  }

  double dt      = (filter_frequency_ > 0.0) ? 1.0 / filter_frequency_ : 0.05;
  double elapsed = (ros::Time::now() - last_command_time_).toSec();

  if (elapsed > velocity_timeout_) {
    if (stopped_) {
      return;
    }
    command_ = geometry_msgs::Twist();
  }

  double cv_x = open_loop_ ? last_cmd_.linear.x  : current_vel_.linear.x;
  double cv_y = open_loop_ ? last_cmd_.linear.y  : current_vel_.linear.y;
  double cv_w = open_loop_ ? last_cmd_.angular.z : current_vel_.angular.z;

  double cmd_x = clamp(command_.linear.x,  min_velocities_[0], max_velocities_[0]);
  double cmd_y = clamp(command_.linear.y,  min_velocities_[1], max_velocities_[1]);
  double cmd_w = clamp(command_.angular.z, min_velocities_[2], max_velocities_[2]);

  if (scale_velocities_) {
    double dx = cmd_x - cv_x;
    double dy = cmd_y - cv_y;
    double dw = cmd_w - cv_w;

    double eta = 1.0;

    auto axisEta = [&](double dv, double lim_pos, double lim_neg) {
      if (dv > lim_pos && std::fabs(dv) > 1e-9) return lim_pos / dv;
      if (dv < lim_neg && std::fabs(dv) > 1e-9) return lim_neg / dv;
      return 1.0;
    };

    double step_x = ((std::fabs(cmd_x) >= std::fabs(cv_x) && cv_x * cmd_x >= 0.0)
      ? max_accels_[0] : -max_decels_[0]) / filter_frequency_;
    double step_y = ((std::fabs(cmd_y) >= std::fabs(cv_y) && cv_y * cmd_y >= 0.0)
      ? max_accels_[1] : -max_decels_[1]) / filter_frequency_;
    double step_w = ((std::fabs(cmd_w) >= std::fabs(cv_w) && cv_w * cmd_w >= 0.0)
      ? max_accels_[2] : -max_decels_[2]) / filter_frequency_;

    eta = std::min(eta, axisEta(dx, step_x, -step_x));
    eta = std::min(eta, axisEta(dy, step_y, -step_y));
    eta = std::min(eta, axisEta(dw, step_w, -step_w));

    cmd_x = cv_x + eta * dx;
    cmd_y = cv_y + eta * dy;
    cmd_w = cv_w + eta * dw;
  }

  geometry_msgs::Twist raw_jerk_msg;
  geometry_msgs::Twist limited_jerk_msg;

  double nv_x, nv_y, nv_w;
  limitBodyJerkAndPreserveDirection(
    cv_x, cv_y, cv_w,
    cmd_x, cmd_y, cmd_w,
    dt,
    nv_x, nv_y, nv_w,
    raw_jerk_msg, limited_jerk_msg);

  geometry_msgs::Twist out;
  out.linear.x  = nv_x;
  out.linear.y  = nv_y;
  out.linear.z  = command_.linear.z;
  out.angular.x = command_.angular.x;
  out.angular.y = command_.angular.y;
  out.angular.z = nv_w;

  last_cmd_ = out;

  bool near_zero =
    std::fabs(out.linear.x)  < 1e-4 &&
    std::fabs(out.linear.y)  < 1e-4 &&
    std::fabs(out.angular.z) < 1e-4;
  stopped_ = near_zero && (elapsed > velocity_timeout_);

  if (std::fabs(out.linear.x)  < deadband_velocities_[0]) out.linear.x  = 0.0;
  if (std::fabs(out.linear.y)  < deadband_velocities_[1]) out.linear.y  = 0.0;
  if (std::fabs(out.angular.z) < deadband_velocities_[2]) out.angular.z = 0.0;

  filtered_cmd_pub_.publish(out);
  raw_jerk_pub_.publish(raw_jerk_msg);
  limited_jerk_pub_.publish(limited_jerk_msg);
}

}  // namespace jerk_reducer