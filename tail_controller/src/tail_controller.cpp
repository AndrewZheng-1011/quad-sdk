#include "tail_controller/tail_controller.h"

TailController::TailController(ros::NodeHandle nh) {
  nh_ = nh;

  nh.param<int>("/tail_controller/tail_type", tail_type_, 0);
  std::cout << "=========================" << std::endl;
  std::cout << "TAIL TYPE: " << tail_type_ << std::endl;
  std::cout << "=========================" << std::endl;

  switch (tail_type_) {
    case NONE:
      // Leg controller
      param_ns_ = "leg";
      break;
    case CENTRALIZED:
      // Centralized tail controller
      param_ns_ = "centralized_tail";
      break;
    case DISTRIBUTED:
      // Distributed tail controller
      param_ns_ = "distributed_tail";
      break;
    case DECENTRALIZED:
      // Decentralized tail controller
      param_ns_ = "decentralized_tail";
      break;
    case OPEN_LOOP:
      param_ns_ = "open_loop_tail";
      break;
    default:
      param_ns_ = "leg";
      break;
  }

  quad_utils::loadROSParam(nh, "tail_controller/tail_num", tail_num_);
  if (tail_num_ > 2) {
    ROS_ERROR_STREAM(
        "Tail number exceeds available option. Tail num of 1 to 2 are "
        "available");
  }

  // Get rosparams
  std::string tail_plan_topic, tail_control_topic, robot_state_topic;
  quad_utils::loadROSParam(nh_, "/topics/control/tail_plan", tail_plan_topic);
  quad_utils::loadROSParam(nh_, "/topics/control/tail_command",
                           tail_control_topic);
  quad_utils::loadROSParam(nh_, "/topics/state/ground_truth",
                           robot_state_topic);
  quad_utils::loadROSParam(nh_, "tail_controller/controller_update_rate",
                           update_rate_);
  quad_utils::loadROSParam(nh_, "tail_controller/" + param_ns_ + "/roll_kp",
                           roll_kp_);
  quad_utils::loadROSParam(nh_, "tail_controller/" + param_ns_ + "/roll_kd",
                           roll_kd_);
  quad_utils::loadROSParam(nh_, "tail_controller/" + param_ns_ + "/pitch_kp",
                           pitch_kp_);
  quad_utils::loadROSParam(nh_, "tail_controller/" + param_ns_ + "/pitch_kd",
                           pitch_kd_);

  // Extra params from open loop tail

  if (param_ns_ == "open_loop_tail") {
    quad_utils::loadROSParam(
        nh_, "tail_controller/" + param_ns_ + "/ff_torque_1", ff_torque_1);
    quad_utils::loadROSParam(
        nh_, "tail_controller/" + param_ns_ + "/ff_torque_2", ff_torque_2);
    quad_utils::loadROSParam(nh_, "tail_controller/" + param_ns_ + "/time_1",
                             time_1);
    quad_utils::loadROSParam(nh_, "tail_controller/" + param_ns_ + "/time_2",
                             time_2);

    // If ran through quad_gazebo_dira, rewrite these params loaded from launch
    // file args
    if (nh_.hasParam("tail_controller/" + param_ns_ + "/param_ff_torque_1")) {
      quad_utils::loadROSParam(
          nh_, "tail_controller/" + param_ns_ + "/param_ff_torque_1",
          ff_torque_1);
    }

    if (nh_.hasParam("tail_controller/" + param_ns_ + "/param_ff_torque_2")) {
      quad_utils::loadROSParam(
          nh_, "tail_controller/" + param_ns_ + "/param_ff_torque_2",
          ff_torque_2);
    }

    if (nh_.hasParam("tail_controller/" + param_ns_ + "/param_time_1")) {
      quad_utils::loadROSParam(
          nh_, "tail_controller/" + param_ns_ + "/param_time_1", time_1);
    }

    if (nh_.hasParam("tail_controller/" + param_ns_ + "/param_time_2")) {
      quad_utils::loadROSParam(
          nh_, "tail_controller/" + param_ns_ + "/param_time_2", time_2);
    }
  }

  // Setup pubs and subs
  tail_control_pub_ =
      nh_.advertise<quad_msgs::LegCommand>(tail_control_topic, 1);
  tail_plan_sub_ =
      nh_.subscribe(tail_plan_topic, 1, &TailController::tailPlanCallback, this,
                    ros::TransportHints().tcpNoDelay(true));
  robot_state_sub_ =
      nh_.subscribe(robot_state_topic, 1, &TailController::robotStateCallback,
                    this, ros::TransportHints().tcpNoDelay(true));
}

void TailController::tailPlanCallback(
    const quad_msgs::LegCommandArray::ConstPtr &msg) {
  last_tail_plan_msg_ = msg;
}

void TailController::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr &msg) {
  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty()) return;

  robot_state_msg_ = msg;
}

void TailController::publishTailCommand() {
  if (robot_state_msg_ == NULL ||
      robot_state_msg_->tail_joints.position.empty()) {
    return;
  }

  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  tail_current_state_ = quad_utils::odomMsgToEigenForTail(*robot_state_msg_);

  quad_msgs::LegCommand msg;
  msg.motor_commands.resize(2);

  // THIS IS WHERE I START RIGHT RANDOM FEEDFORWARD TAIL CONTROL
  // TODO(AZ): WORK ON OPEN LOOP CONTROL
  // SEEMS LIKE UNDERSTANDING THE STATE HERE IS THE BEST WAY
  // CAN WORK WITH THIS
  // SOMEHOW GET THE SIM_TIME AND EXECUTE DURING SIM TIME WHEN FALLING
  if (param_ns_ == "decentralized_tail") {
    // Feedback tail
    ROS_WARN_ONCE("DECENTRALIZED TAIL FB CONTROL");
    ROS_WARN_THROTTLE(0.25, "Pos setpoint %0.2f", -current_state_(3));
    msg.motor_commands.at(0).pos_setpoint =
        -current_state_(3);  // Original: -current_state_(3);
    msg.motor_commands.at(0).vel_setpoint = 5 * current_state_(9);
    msg.motor_commands.at(0).torque_ff = 0;
    msg.motor_commands.at(0).kp = roll_kp_;
    msg.motor_commands.at(0).kd = roll_kd_;
    // TODO(AZ): Make this robust depending on number of tail joints
    msg.motor_commands.at(1).pos_setpoint = -current_state_(4);
    msg.motor_commands.at(1).vel_setpoint = 5 * current_state_(10);
    msg.motor_commands.at(1).torque_ff = 0;
    msg.motor_commands.at(1).kp = pitch_kp_;
    msg.motor_commands.at(1).kd = pitch_kd_;
  } else if (param_ns_ == "open_loop_tail") {
    // Open Loop Tail
    ROS_WARN_ONCE("OPENLOOP TAIL");
    ROS_WARN_THROTTLE(0.25, "Pos setpoint %0.2f", -current_state_(3));
    ROS_WARN_THROTTLE(0.25, "time: %0.2f", ros::Time::now().toSec());

    msg.motor_commands.at(0).pos_setpoint =
        -current_state_(3);  // Original: -current_state_(3);
    msg.motor_commands.at(0).vel_setpoint = 5 * current_state_(9);
    msg.motor_commands.at(0).torque_ff = 0;
    msg.motor_commands.at(0).kp = roll_kp_;
    msg.motor_commands.at(0).kd = roll_kd_;
    msg.motor_commands.at(1).pos_setpoint = -current_state_(4);
    msg.motor_commands.at(1).vel_setpoint = 5 * current_state_(10);
    msg.motor_commands.at(1).torque_ff = 0;
    msg.motor_commands.at(1).kp = pitch_kp_;
    msg.motor_commands.at(1).kd = pitch_kd_;

    if (ros::Time::now().toSec() > time_1 &&
        ros::Time::now().toSec() < (time_1 + 0.20)) {
      ROS_WARN_ONCE("FF TORQUE of %0.2f", ff_torque_1);
      msg.motor_commands.at(0).torque_ff = ff_torque_1;
      msg.motor_commands.at(1).torque_ff = 0;

      msg.motor_commands.at(0).pos_setpoint = 0;
      msg.motor_commands.at(0).vel_setpoint = 0;
      msg.motor_commands.at(0).kp = 0;
      msg.motor_commands.at(0).kd = 0;
      // TODO(AZ): Make this robust depending on number of tail joints
      // Seems like no need... works anyways... idk why & don't need to figure
      // out for this project
      msg.motor_commands.at(1).pos_setpoint = 0;
      msg.motor_commands.at(1).vel_setpoint = 0;
      msg.motor_commands.at(1).kp = 0;
      msg.motor_commands.at(1).kd = 0;
    }

    if (ros::Time::now().toSec() > time_2 &&
        ros::Time::now().toSec() < (time_2 + 0.20)) {
      ROS_WARN_ONCE("FF TORQUE of %0.2f", ff_torque_2);
      msg.motor_commands.at(0).torque_ff = 0;
      msg.motor_commands.at(1).torque_ff = ff_torque_2;

      msg.motor_commands.at(0).pos_setpoint = 0;
      msg.motor_commands.at(0).vel_setpoint = 0;
      msg.motor_commands.at(0).kp = 0;
      msg.motor_commands.at(0).kd = 0;
      // TODO(AZ): Make this robust depending on number of tail joints
      msg.motor_commands.at(1).pos_setpoint = 0;
      msg.motor_commands.at(1).vel_setpoint = 0;
      msg.motor_commands.at(1).kp = 0;
      msg.motor_commands.at(1).kd = 0;
    }
  } else if (last_tail_plan_msg_ == NULL) {
    // No tail plan yet
    msg.motor_commands.at(0).pos_setpoint = 0;
    msg.motor_commands.at(0).vel_setpoint = 0;
    msg.motor_commands.at(0).torque_ff = 0;
    msg.motor_commands.at(0).kp = roll_kp_;
    msg.motor_commands.at(0).kd = roll_kd_;

    msg.motor_commands.at(1).pos_setpoint = 0;
    msg.motor_commands.at(1).vel_setpoint = 0;
    msg.motor_commands.at(1).torque_ff = 0;
    msg.motor_commands.at(1).kp = pitch_kp_;
    msg.motor_commands.at(1).kd = pitch_kd_;
  } else {
    double t_interp;
    int current_plan_index;
    double t_now = ros::Time::now().toSec();

    // Interpolate the local plan to get the reference state and ff GRF
    for (int i = 0; i < last_tail_plan_msg_->leg_commands.size() - 1; i++) {
      if ((t_now >=
           (last_tail_plan_msg_->leg_commands[i].header.stamp).toSec()) &&
          (t_now <
           (last_tail_plan_msg_->leg_commands[i + 1].header.stamp).toSec())) {
        // Record the current plan index
        current_plan_index = i;

        t_interp =
            (t_now -
             (last_tail_plan_msg_->leg_commands[i].header.stamp).toSec()) /
            (last_tail_plan_msg_->leg_commands[i + 1].header.stamp.toSec() -
             last_tail_plan_msg_->leg_commands[i].header.stamp.toSec());

        break;
      }
    }

    // If we are out of the plan interval
    if (current_plan_index + 1 > last_tail_plan_msg_->leg_commands.size() - 1) {
      ROS_ERROR("Tail controller node couldn't find the correct ref state!");

      msg.motor_commands.at(0).pos_setpoint =
          last_tail_plan_msg_->leg_commands.back()
              .motor_commands[0]
              .pos_setpoint;
      msg.motor_commands.at(0).vel_setpoint =
          last_tail_plan_msg_->leg_commands.back()
              .motor_commands[0]
              .vel_setpoint;
      msg.motor_commands.at(0).torque_ff =
          last_tail_plan_msg_->leg_commands.back().motor_commands[0].torque_ff;
      msg.motor_commands.at(0).kp = roll_kp_;
      msg.motor_commands.at(0).kd = roll_kd_;

      msg.motor_commands.at(1).pos_setpoint =
          last_tail_plan_msg_->leg_commands.back()
              .motor_commands[1]
              .pos_setpoint;
      msg.motor_commands.at(1).vel_setpoint =
          last_tail_plan_msg_->leg_commands.back()
              .motor_commands[1]
              .vel_setpoint;
      msg.motor_commands.at(1).torque_ff =
          last_tail_plan_msg_->leg_commands.back().motor_commands[1].torque_ff;
      msg.motor_commands.at(1).kp = pitch_kp_;
      msg.motor_commands.at(1).kd = pitch_kd_;
    } else {
      // Interpolate between two plans
      msg.motor_commands.at(0).pos_setpoint = math_utils::lerp(
          last_tail_plan_msg_->leg_commands[current_plan_index]
              .motor_commands[0]
              .pos_setpoint,
          last_tail_plan_msg_->leg_commands[current_plan_index + 1]
              .motor_commands[0]
              .pos_setpoint,
          t_interp);
      msg.motor_commands.at(0).vel_setpoint = math_utils::lerp(
          last_tail_plan_msg_->leg_commands[current_plan_index]
              .motor_commands[0]
              .vel_setpoint,
          last_tail_plan_msg_->leg_commands[current_plan_index + 1]
              .motor_commands[0]
              .vel_setpoint,
          t_interp);
      msg.motor_commands.at(0).torque_ff =
          last_tail_plan_msg_->leg_commands[current_plan_index]
              .motor_commands[0]
              .torque_ff;
      msg.motor_commands.at(0).kp = roll_kp_;
      msg.motor_commands.at(0).kd = roll_kd_;

      msg.motor_commands.at(1).pos_setpoint = math_utils::lerp(
          last_tail_plan_msg_->leg_commands[current_plan_index]
              .motor_commands[1]
              .pos_setpoint,
          last_tail_plan_msg_->leg_commands[current_plan_index + 1]
              .motor_commands[1]
              .pos_setpoint,
          t_interp);
      msg.motor_commands.at(1).vel_setpoint = math_utils::lerp(
          last_tail_plan_msg_->leg_commands[current_plan_index]
              .motor_commands[1]
              .vel_setpoint,
          last_tail_plan_msg_->leg_commands[current_plan_index + 1]
              .motor_commands[1]
              .vel_setpoint,
          t_interp);
      msg.motor_commands.at(1).torque_ff =
          last_tail_plan_msg_->leg_commands[current_plan_index]
              .motor_commands[1]
              .torque_ff;
      msg.motor_commands.at(1).kp = pitch_kp_;
      msg.motor_commands.at(1).kd = pitch_kd_;
    }
  }

  // Record control history
  for (size_t i = 0; i < 2; i++) {
    double pos_component =
        msg.motor_commands.at(i).kp *
        (msg.motor_commands.at(i).pos_setpoint - tail_current_state_(i));
    double vel_component =
        msg.motor_commands.at(i).kd *
        (msg.motor_commands.at(i).vel_setpoint - tail_current_state_(2 + i));
    double fb_component = pos_component + vel_component;
    double effort = fb_component + msg.motor_commands.at(i).torque_ff;
    double fb_ratio =
        abs(fb_component) /
        (abs(fb_component) + abs(msg.motor_commands.at(i).torque_ff));

    msg.motor_commands.at(i).pos_component = pos_component;
    msg.motor_commands.at(i).vel_component = vel_component;
    msg.motor_commands.at(i).fb_component = fb_component;
    msg.motor_commands.at(i).effort = effort;
    msg.motor_commands.at(i).fb_ratio = fb_ratio;
  }

  msg.header.stamp = ros::Time::now();
  tail_control_pub_.publish(msg);
}

void TailController::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    ros::spinOnce();
    this->publishTailCommand();
    r.sleep();
  }
}
