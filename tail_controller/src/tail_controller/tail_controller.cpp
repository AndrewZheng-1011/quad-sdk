#include "tail_controller/tail_controller.h"

TailController::TailController(ros::NodeHandle nh) {
	nh_ = nh;

  // Get rosparams
  std::string leg_control_topic = "/spirit/joint_controller/tail_command";
  // spirit_utils::loadROSParam(nh_,"topics/joint_command",leg_control_topic);
  // spirit_utils::loadROSParam(nh_,"open_loop_controller/control_mode_topic",control_mode_topic);
  nh.param<double>("open_loop_controller/update_rate",update_rate_, 50);
  // spirit_utils::loadROSParam(nh_,"open_loop_controller/stand_angles",stand_joint_angles_);
  // spirit_utils::loadROSParam(nh_,"open_loop_controller/stand_kp",stand_kp_);
  // spirit_utils::loadROSParam(nh_,"open_loop_controller/stand_kd",stand_kd_);
  // spirit_utils::loadROSParam(nh_,"open_loop_controller/walk_kp",walk_kp_);
  // spirit_utils::loadROSParam(nh_,"open_loop_controller/walk_kd",walk_kd_);
  // spirit_utils::loadROSParam(nh_,"open_loop_controller/leg_phases",leg_phases_);
  // spirit_utils::loadROSParam(nh_,"open_loop_controller/use_diff_for_velocity",use_diff_for_velocity_);

  // Start sitting
  // control_mode_ = 0;

  // Interpolate between waypoints
  this->setupTrajectory();
  
  // Setup pubs and subs
  joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommand>(leg_control_topic,1);
  // control_mode_sub_ = nh_.subscribe(control_mode_topic,1,&OpenLoopController::controlModeCallback, this);
}

// void TailController::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
//   if (0 <= msg->data && msg->data <= 2)
//   {
//     control_mode_ = msg->data;
//   }
// }

void TailController::setupTrajectory()
{
  std::vector<double> ts = {0.075, 0.075, 0.075, 0.075, 0.075, 0.075, 0.075, 0.075}; 
  std::vector<double> xs = {0, 3.14159/8, 3.14159/4, 3.14159/8, 0, -3.14159/8, -3.14159/4, -3.14159/8}; 
  std::vector<double> ys = {0, 3.14159/4, 0, -3.14159/4, 0, 3.14159/4, 0, -3.14159/4}; 

  interp_dt_ = 0.005;

  // Interpolate between points with fixed dt
  double t_run = 0;
  for (int i = 0; i < ts.size(); ++i)
  {
    double xfirst = xs.at(i);
    double yfirst = ys.at(i);
    double t0 = t_run; 

    int idx_last = i+1;
    if (i == ts.size() - 1) idx_last = 0;

    double xlast = xs.at(idx_last);
    double ylast = ys.at(idx_last);
    double t1 = t_run + ts.at(i);

    double t = t0+interp_dt_;
    while(t <= t1)
    {
      double r = (t-t0)/(t1-t0); // ratio of xfirst to xlast to use
      double x = xfirst + r*(xlast-xfirst);
      double y = yfirst + r*(ylast-yfirst);
      target_pts_.push_back(std::make_pair(x,y));
      target_times_.push_back(t);
      t += interp_dt_;
    }
    t_run += ts.at(i);
  }
  t_cycle_ = target_times_.back();
}

void TailController::sendJointPositions(double &elapsed_time)
{
	spirit_msgs::LegCommand msg;

  // Get relative time through gait for this leg (inc phase info)
  double t = fmodf(elapsed_time + t_cycle_,t_cycle_);

  // Find target position from precomputed trajectory
  auto it = std::upper_bound(target_times_.begin(), target_times_.end(),t);
  int target_idx = it - target_times_.begin();

  // Convert to joint angles
  std::pair<double,double> hip_knee_angs = target_pts_.at(target_idx);

  // Find target velocity from central difference of precomputed trajectory
  int prev_idx = target_idx == 0 ? target_pts_.size() - 1 : target_idx - 1;
  int next_idx = target_idx == target_pts_.size() - 1? 0 : target_idx + 1;
  std::pair<double,double> prev_hip_knee_angs = target_pts_.at(prev_idx);
  std::pair<double,double> next_hip_knee_angs = target_pts_.at(next_idx);
  double hip_vel = (next_hip_knee_angs.first - prev_hip_knee_angs.first)/(2*interp_dt_);
  double knee_vel = (next_hip_knee_angs.second - prev_hip_knee_angs.second)/(2*interp_dt_);

  // Fill out motor command
  msg.motor_commands.resize(2);

  msg.motor_commands.at(0).pos_setpoint = hip_knee_angs.first;
  msg.motor_commands.at(0).kp = 50;
  msg.motor_commands.at(0).kd = 5;
  msg.motor_commands.at(0).vel_setpoint = hip_vel;
  msg.motor_commands.at(0).torque_ff = 0;

  msg.motor_commands.at(1).pos_setpoint = hip_knee_angs.second;
  msg.motor_commands.at(1).kp = 50;
  msg.motor_commands.at(1).kd = 5;
  msg.motor_commands.at(1).vel_setpoint = knee_vel;
  msg.motor_commands.at(1).torque_ff = 0;

  msg.header.stamp = ros::Time::now();
  joint_control_pub_.publish(msg);
}

void TailController::spin() {
  double start_time = ros::Time::now().toSec();
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    double elapsed_time = ros::Time::now().toSec() - start_time;
    this->sendJointPositions(elapsed_time);
    ros::spinOnce();
    r.sleep();
  }
}