#include "ekf_estimator/ekf_estimator.h"

EKFEstimator::EKFEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, contact_topic,
      state_estimate_topic;
  nh.param<std::string>("topics/state/joints", joint_encoder_topic,
                        "/state/joints");
  nh.param<std::string>("topics/state/imu", imu_topic, "/state/imu");
  nh.param<std::string>("topics/state/estimate", state_estimate_topic,
                        "/state/estimate");
  nh.param<std::string>("topics/contact_mode", contact_topic, "/contact_mode");
  nh.param<double>("ekf_estimator/update_rate", update_rate_, 200);
  nh.param<double>("ekf_estimator/joint_state_max_time",
                   joint_state_msg_time_diff_max_, 20);

  // Load initial IMU bias
  nh.getParam("/ekf_estimator/bias_x", bias_x_);
  nh.getParam("/ekf_estimator/bias_y", bias_y_);
  nh.getParam("/ekf_estimator/bias_z", bias_z_);
  nh.getParam("/ekf_estimator/bias_r", bias_r_);
  nh.getParam("/ekf_estimator/bias_p", bias_p_);
  nh.getParam("/ekf_estimator/bias_w", bias_w_);

  // Load noise terms
  nh.getParam("/ekf_estimator/na", na_);
  nh.getParam("/ekf_estimator/ng", ng_);
  nh.getParam("/ekf_estimator/ba", ba_);
  nh.getParam("/ekf_estimator/bg", bg_);
  nh.getParam("/ekf_estimator/nf", nf_);
  nh.getParam("/ekf_estimator/nfk", nfk_);
  nh.getParam("/ekf_estimator/ne", ne_);

  // load ground_truth state rosparams and setup subs
  std::string state_ground_truth_topic;
  nh.param<std::string>("topic/state/ground_truth", state_ground_truth_topic,
                        "/state/ground_truth");
  state_ground_truth_sub_ = nh_.subscribe(
      state_ground_truth_topic, 1, &EKFEstimator::groundtruthCallback, this);

  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic, 1,
                                     &EKFEstimator::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic, 1, &EKFEstimator::imuCallback, this);
  contact_sub_ =
      nh_.subscribe(contact_topic, 1, &EKFEstimator::contactCallback, this);
  state_estimate_pub_ =
      nh_.advertise<quad_msgs::RobotState>(state_estimate_topic, 1);

  // QuadKD class
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
}

void EKFEstimator::groundtruthCallback(
    const quad_msgs::RobotState::ConstPtr& msg) {
  last_state_msg_ = msg;
}

void EKFEstimator::jointEncoderCallback(
    const sensor_msgs::JointState::ConstPtr& msg) {
  last_joint_state_msg_ = msg;
}

void EKFEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = msg;
}

void EKFEstimator::contactCallback(
    const quad_msgs::ContactMode::ConstPtr& msg) {
  last_contact_msg_ = msg;
}

void EKFEstimator::setX(Eigen::VectorXd Xin) { X = Xin; }

void EKFEstimator::setlast_X(Eigen::VectorXd Xin) { last_X = Xin; }

void EKFEstimator::setP(Eigen::MatrixXd Pin) { P = Pin; }

Eigen::VectorXd EKFEstimator::getX() { return X; }

Eigen::VectorXd EKFEstimator::getlastX() { return last_X; }

Eigen::VectorXd EKFEstimator::getX_pre() { return X_pre; }

quad_msgs::RobotState EKFEstimator::StepOnce() {
  // Record start time of function, used in verifying messages are not out of
  // date and in timing function
  ros::Time start_time = ros::Time::now();

  // Create skeleton message to send out
  quad_msgs::RobotState new_state_est;

  // calculate dt
  double dt = (start_time - last_time).toSec();
  last_time = start_time;

  // std::cout << "this is dt" << dt << std::endl;

  /// Collect and Process Data
  // IMU reading linear acceleration
  Eigen::VectorXd fk = Eigen::VectorXd::Zero(3);
  // IMU reading angular acceleration
  Eigen::VectorXd wk = Eigen::VectorXd::Zero(3);
  // IMU orientation (w, x, y, z)
  Eigen::Quaterniond qk(1, 0, 0, 0);
  // if there is good imu data: read data from bag file
  this->readIMU(last_imu_msg_, fk, wk, qk);

  // Joint data reading 3 joints * 4 legs
  Eigen::VectorXd jk = Eigen::VectorXd::Zero(12);
  this->readJointEncoder(last_joint_state_msg_, jk);
  std::vector<double> jkVector(jk.data(), jk.data() + jk.rows() * jk.cols());

  /// Prediction Step
  // std::cout << "this is X before" << X.transpose() << std::endl;
  this->predict(dt, fk, wk, qk);
  // std::cout << "this is X predict" << X.transpose() << std::endl;

  // for testing prediction step
  X = X_pre;
  P = P_pre;
  last_X = X;

  /// Update Step
  // this->update(jk);
  // std::cout << "this is X update" << X.transpose() << std::endl;

  /// publish new message
  new_state_est.header.stamp = ros::Time::now();

  // body

  new_state_est.body.pose.orientation.w = X[6];
  new_state_est.body.pose.orientation.x = X[7];
  new_state_est.body.pose.orientation.y = X[8];
  new_state_est.body.pose.orientation.z = X[9];

  new_state_est.body.pose.position.x = X[0];
  new_state_est.body.pose.position.y = X[1];
  new_state_est.body.pose.position.z = X[2];

  new_state_est.body.twist.linear.x = X[3];
  new_state_est.body.twist.linear.y = X[4];
  new_state_est.body.twist.linear.z = X[5];

  // joint
  new_state_est.joints.header.stamp = ros::Time::now();
  // '8', '0', '1', '9', '2', '3', '10', '4', '5', '11', '6', '7'
  new_state_est.joints.name = {"8",  "0", "1", "9",  "2", "3",
                               "10", "4", "5", "11", "6", "7"};
  new_state_est.joints.position = jkVector;
  new_state_est.joints.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  new_state_est.joints.effort = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // feet

  return new_state_est;
}

void EKFEstimator::predict(const double& dt, const Eigen::VectorXd& fk,
                           const Eigen::VectorXd& wk,
                           const Eigen::Quaterniond& qk) {
  // calculate rotational matrix from world frame to body frame
  Eigen::Matrix3d C = (qk.toRotationMatrix()).transpose();

  // Collect states info from previous state vector
  Eigen::VectorXd r = last_X.segment(0, 3);
  Eigen::VectorXd v = last_X.segment(3, 3);
  Eigen::VectorXd q = last_X.segment(6, 4);
  Eigen::VectorXd p = last_X.segment(10, 12);
  Eigen::VectorXd bf = last_X.segment(22, 3);
  Eigen::VectorXd bw = last_X.segment(25, 3);
  // calculate linear acceleration set z acceleration to -9.8 for now
  // a is the corrected IMU linear acceleration (fk hat)
  Eigen::VectorXd a = Eigen::VectorXd::Zero(3);
  a = fk - bf;
  a[2] = -9.8;

  g = Eigen::VectorXd::Zero(3);
  g[2] = 9.81;
  // calculate angular acceleration
  // w is the corrected IMU angular acceleration (wk hat)
  Eigen::VectorXd w = Eigen::VectorXd::Zero(3);
  w = wk - bw;

  // state prediction X_pre
  X_pre = Eigen::VectorXd::Zero(num_state);
  X_pre.segment(0, 3) = r + dt * v + dt * dt * 0.5 * (C.transpose() * a + g);
  X_pre.segment(3, 3) = v + dt * (C.transpose() * a + g);
  // quaternion updates
  Eigen::VectorXd wdt = dt * w;
  Eigen::VectorXd q_pre = this->quaternionDynamics(wdt, q);
  // Eigen::VectorXd q_pre = q;

  X_pre.segment(6, 4) = q_pre;
  X_pre.segment(10, 12) = p;
  X_pre.segment(22, 3) = bf;
  X_pre.segment(25, 3) = bw;

  // Linearized Dynamics Matrix
  F = Eigen::MatrixXd::Identity(num_cov, num_cov);
  F.block<3, 3>(0, 3) = dt * Eigen::MatrixXd::Identity(3, 3);

  Eigen::MatrixXd fkskew = this->calcSkewsym(a);
  Eigen::MatrixXd r0 = this->calcRodrigues(dt, w, 0);
  Eigen::MatrixXd r1 = this->calcRodrigues(dt, w, 1);
  Eigen::MatrixXd r2 = this->calcRodrigues(dt, w, 2);
  Eigen::MatrixXd r3 = this->calcRodrigues(dt, w, 3);

  F.block<3, 3>(0, 6) = -(dt * dt / 2) * C.transpose() * fkskew;
  F.block<3, 3>(0, 21) = -(dt * dt / 2) * C.transpose();
  F.block<3, 3>(3, 6) = -dt * C.transpose() * fkskew;
  F.block<3, 3>(3, 21) = -dt * C.transpose();
  F.block<3, 3>(6, 6) = r0.transpose();
  F.block<3, 3>(6, 24) = -1 * r1.transpose();

  // Discrete Process Noise Covariance Matrix
  Q = Eigen::MatrixXd::Zero(num_cov, num_cov);
  Q.block<3, 3>(0, 0) =
      (pow(dt, 3) / 3) * noise_acc + (pow(dt, 5) / 20) * bias_acc;
  Q.block<3, 3>(0, 3) =
      (pow(dt, 2) / 2) * noise_acc + (pow(dt, 4) / 8) * bias_acc;
  Q.block<3, 3>(0, 21) = -1 * (pow(dt, 3) / 6) * C.transpose() * bias_acc;
  Q.block<3, 3>(3, 0) =
      (pow(dt, 2) / 2) * noise_acc + (pow(dt, 4) / 8) * bias_acc;
  Q.block<3, 3>(3, 3) = dt * noise_acc + (pow(dt, 3) / 3) * bias_acc;
  Q.block<3, 3>(3, 21) = -1 * (pow(dt, 2) / 2) * C.transpose() * bias_acc;
  Q.block<3, 3>(6, 6) = dt * noise_gyro + (r3 + r3.transpose()) * bias_gyro;
  Q.block<3, 3>(6, 24) = -r2.transpose() * bias_gyro;
  for (int i = 0; i < num_feet; i++) {
    Q.block<3, 3>(9 + i * 3, 9 + i * 3) = dt * C.transpose() * noise_feet * C;
    // determine foot contact and set noise

    if (p[i * 3 + 2] < 0.02) {
      Q.block<3, 3>(9 + i * 3, 9 + i * 3) = dt * C.transpose() * noise_feet * C;
    } else {
      Q.block<3, 3>(9 + i * 3, 9 + i * 3) =
          1000 * dt * C.transpose() * noise_feet * C;
    }
  }

  Q.block<3, 3>(21, 0) = -1 * (pow(dt, 3) / 6) * bias_acc * C;
  Q.block<3, 3>(21, 3) = -1 * (pow(dt, 2) / 2) * bias_acc * C;
  Q.block<3, 3>(21, 21) = dt * bias_acc;
  Q.block<3, 3>(24, 6) = -1 * bias_gyro * r2;
  Q.block<3, 3>(24, 24) = dt * bias_gyro;
  // Q = Eigen::MatrixXd::Zero(num_cov, num_cov);

  // Covariance update
  P_pre = (F * P * F.transpose()) + Q;

  // std::cout << "this is Q " << Q << std::endl;
  // std::cout << "this is P_pre " << P_pre << std::endl;
}

void EKFEstimator::update(const Eigen::VectorXd& jk) {
  // debug for update step, set the predicted state to be ground truth:
  // if (last_state_msg_ != NULL) {
  //   X_pre << (*last_state_msg_).body.pose.position.x,
  //       (*last_state_msg_).body.pose.position.y,
  //       (*last_state_msg_).body.pose.position.z,
  //       (*last_state_msg_).body.twist.linear.x,
  //       (*last_state_msg_).body.twist.linear.y,
  //       (*last_state_msg_).body.twist.linear.z,
  //       (*last_state_msg_).body.pose.orientation.w,
  //       (*last_state_msg_).body.pose.orientation.x,
  //       (*last_state_msg_).body.pose.orientation.y,
  //       (*last_state_msg_).body.pose.orientation.z,
  //       (*last_state_msg_).feet.feet[0].position.x,
  //       (*last_state_msg_).feet.feet[0].position.y,
  //       (*last_state_msg_).feet.feet[0].position.z,
  //       (*last_state_msg_).feet.feet[1].position.x,
  //       (*last_state_msg_).feet.feet[1].position.y,
  //       (*last_state_msg_).feet.feet[1].position.z,
  //       (*last_state_msg_).feet.feet[2].position.x,
  //       (*last_state_msg_).feet.feet[2].position.y,
  //       (*last_state_msg_).feet.feet[2].position.z,
  //       (*last_state_msg_).feet.feet[3].position.x,
  //       (*last_state_msg_).feet.feet[3].position.y,
  //       (*last_state_msg_).feet.feet[3].position.z, X_pre.segment(22, 3),
  //       X_pre.segment(25, 3);
  // }

  // Collect states info from predicted state vector
  Eigen::VectorXd r_pre = X_pre.segment(0, 3);
  Eigen::VectorXd v_pre = X_pre.segment(3, 3);
  Eigen::VectorXd q_pre = X_pre.segment(6, 4);
  Eigen::VectorXd p_pre = X_pre.segment(10, 12);
  Eigen::VectorXd bf_pre = X_pre.segment(22, 3);
  Eigen::VectorXd bw_pre = X_pre.segment(25, 3);

  Eigen::Quaterniond quaternion_pre(q_pre[0], q_pre[1], q_pre[2], q_pre[3]);
  quaternion_pre.normalize();
  Eigen::Matrix3d C_pre = (quaternion_pre.toRotationMatrix()).transpose();

  // Measured feet positions in the body frame
  Eigen::VectorXd s = Eigen::VectorXd::Zero(3 * num_feet);
  // foot index i:(0 = FL, 1 = BL, 2 = FR, 3 = BR)
  // std::cout << "this is jk value " << jk << std::endl;
  // throw std::runtime_error(" runtime error STOP");
  for (int i = 0; i < num_feet; i++) {
    Eigen::Vector3d joint_state_i;
    // FL: 0 1 2 , BL: 3 4 5, FR: 6 7 8, BR: 9 10 11
    joint_state_i = jk.segment(3 * i, 3);

    Eigen::Vector3d toe_body_pos;
    quadKD_->bodyToFootFKBodyFrame(i, joint_state_i, toe_body_pos);
    s.segment(i * 3, 3) = toe_body_pos;
  }

  // std::cout << "measured foot positions" << s << std::endl;

  // std::cout << "C_pre" << C_pre << std::endl;
  // std::cout << "p_pre" << p_pre << std::endl;
  // std::cout << "r_pre" << r_pre << std::endl;
  // measurement residual (12 * 1)
  Eigen::VectorXd y = Eigen::VectorXd::Zero(num_measure);
  for (int i = 0; i < num_feet; i++) {
    // predicted value of the foot positions
    Eigen::VectorXd foot_temp = C_pre * (p_pre.segment(i * 3, 3) - r_pre);
    y.segment(i * 3, 3) = s.segment(i * 3, 3) - foot_temp;
  }
  // std::cout << "this is the residual " << y.transpose() << std::endl;
  // std::cout << "maxium residual is " << y.maxCoeff() << std::endl;

  // Measurement jacobian (12 * 27)
  H = Eigen::MatrixXd::Zero(num_measure, num_cov);
  H.block<3, 3>(0, 0) = -C_pre;
  H.block<3, 3>(3, 0) = -C_pre;
  H.block<3, 3>(6, 0) = -C_pre;
  H.block<3, 3>(9, 0) = -C_pre;

  for (int i = 0; i < num_feet; i++) {
    Eigen::VectorXd vtemp = C_pre * (p_pre.segment(i * 3, 3) - r_pre);
    H.block<3, 3>(i * 3, 6) = this->calcSkewsym(vtemp);
    H.block<3, 3>(i * 3, 9 + i * 3) = C_pre;
  }

  // Measurement Noise Matrix (12 * 12)
  R = 0.0001 * Eigen::MatrixXd::Identity(num_measure, num_measure);

  // // Define vectors for state positions
  // Eigen::VectorXd state_positions(18);
  // // Load state positions
  // state_positions << jk, r_pre, v_pre;
  // // Compute jacobian
  // Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
  // quadKD_->getJacobianBodyAngVel(state_positions, jacobian);

  // for (int i = 0; i < num_feet; i++) {
  //   Eigen::MatrixXd jtemp = jacobian.block<3, 3>(i * 3, i * 3);
  //   R.block<3, 3>(i * 3, i * 3) =
  //       noise_fk + jtemp * noise_encoder * jtemp.transpose();
  // }

  // update Covariance (12 * 12)
  Eigen::MatrixXd S = H * P_pre * H.transpose() + R;
  // K (27 * 12)
  Eigen::MatrixXd K = P_pre * H.transpose() * S.inverse();
  Eigen::VectorXd delta_X = K * y;

  // std::cout << "kalmin gain " << K << std::endl;
  // std::cout << "delta x " << delta_X.transpose() << std::endl;

  P = P_pre - K * H * P_pre;

  // update state
  X.segment(0, 3) = r_pre + delta_X.segment(0, 3);
  X.segment(3, 3) = v_pre + delta_X.segment(3, 3);
  Eigen::VectorXd delta_q = delta_X.segment(6, 3);
  Eigen::VectorXd q_upd = this->quaternionDynamics(delta_q, q_pre);
  X.segment(6, 4) = q_upd;
  X.segment(10, num_feet * 3) = p_pre + delta_X.segment(9, num_feet * 3);
  X.segment(22, 3) = bf_pre + delta_X.segment(21, 3);
  X.segment(25, 3) = bw_pre + delta_X.segment(24, 3);
  // set current state value to previous statex
  last_X = X;
}

void EKFEstimator::readIMU(const sensor_msgs::Imu::ConstPtr& last_imu_msg_,
                           Eigen::VectorXd& fk, Eigen::VectorXd& wk,
                           Eigen::Quaterniond& qk) {
  if (last_imu_msg_ != NULL) {
    fk << (*last_imu_msg_).linear_acceleration.x,
        (*last_imu_msg_).linear_acceleration.y,
        (*last_imu_msg_).linear_acceleration.z;

    wk << (*last_imu_msg_).angular_velocity.x,
        (*last_imu_msg_).angular_velocity.y,
        (*last_imu_msg_).angular_velocity.z;

    qk.w() = (*last_imu_msg_).orientation.w;
    qk.x() = (*last_imu_msg_).orientation.x;
    qk.y() = (*last_imu_msg_).orientation.y;
    qk.z() = (*last_imu_msg_).orientation.z;
    qk.normalize();
  }
}

void EKFEstimator::readJointEncoder(
    const sensor_msgs::JointState::ConstPtr& last_joint_state_msg_,
    Eigen::VectorXd& jk) {
  if (last_joint_state_msg_ != NULL) {
    for (int i = 0; i < 12; i++) {
      jk[i] = (*last_joint_state_msg_).position[i];
    }
  }
}

Eigen::VectorXd EKFEstimator::quaternionDynamics(const Eigen::VectorXd& wdt,
                                                 const Eigen::VectorXd& q2v) {
  Eigen::VectorXd output = Eigen::VectorXd::Zero(4);

  double angle = wdt.norm();
  Eigen::Vector3d axis;
  if (angle == 0) {
    axis = Eigen::VectorXd::Zero(3);
  } else {
    axis = wdt / angle;
  }
  Eigen::Vector3d q_xyz = sin(angle / 2) * axis;
  double q_w = cos(angle / 2);
  Eigen::Quaterniond q1(q_w, q_xyz[0], q_xyz[1], q_xyz[2]);

  Eigen::Quaterniond q2(q2v[0], q2v[1], q2v[2], q2v[3]);
  q1.normalize();
  q2.normalize();
  Eigen::Quaterniond q3 = q1 * q2;
  q3.normalize();
  output << q3.w(), q3.x(), q3.y(), q3.z();
  return output;
}

Eigen::MatrixXd EKFEstimator::calcSkewsym(const Eigen::VectorXd& w) {
  Eigen::MatrixXd output = Eigen::MatrixXd::Zero(3, 3);
  output << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return output;
}

Eigen::MatrixXd EKFEstimator::calcRodrigues(const double& dt,
                                            const Eigen::VectorXd& w,
                                            const int& sub) {
  Eigen::MatrixXd output = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd wdt = dt * w;
  double ang = wdt.norm();
  Eigen::VectorXd axis;
  if (ang == 0) {
    axis = Eigen::VectorXd::Zero(3);
  } else {
    axis = wdt / ang;
  }
  Eigen::MatrixXd w_cap = calcSkewsym(axis);
  switch (sub) {
    case 0:
      output = output + sin(ang) * w_cap + (1 - cos(ang)) * (w_cap * w_cap);
      break;

    case 1:
      if (ang == 0) {
        break;
      } else {
        output = output + (1 - cos(ang)) * (w_cap / ang) +
                 (ang - sin(ang)) * (w_cap * w_cap) / ang;
        break;
      }

    case 2:
      if (ang == 0) {
        break;
      } else {
        output = output + (ang - sin(ang)) * (w_cap / (ang * ang)) +
                 (((cos(ang) - 1) + (pow(ang, 2) / 2)) / (ang * ang)) *
                     (w_cap * w_cap);
        break;
      }

    case 3:
      if (ang == 0) {
        break;
      } else {
        output = output +
                 (cos(ang) - 1 + (pow(ang, 2) / 2)) / (pow(ang, 3)) * w_cap +
                 ((sin(ang) - ang + (pow(ang, 3) / 6)) / (pow(ang, 3)) *
                  (w_cap * w_cap));
        break;
      }
    default:
      break;
  }
  output = pow(dt, sub) * output;
  return output;
}

void EKFEstimator::setNoise() {
  this->noise_acc = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_gyro = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->bias_acc = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->bias_gyro = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_feet = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_fk = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_encoder = 0.001;
  return;
}

void EKFEstimator::spin() {
  ros::Rate r(update_rate_);

  // initial state
  X0 = Eigen::VectorXd::Zero(num_state);
  X = X0;
  last_X = X0;
  X_pre = X0;

  // initial state covariance matrix
  P0 = 0.001 * Eigen::MatrixXd::Identity(num_cov, num_cov);
  P = P0;

  // set noise
  this->setNoise();

  // set gravity
  g = Eigen::VectorXd::Zero(3);
  g[2] = 9.8;

  while (ros::ok()) {
    // Collect new messages on subscriber topics
    ros::spinOnce();

    // if there are no encoder or imu reading: don't publish anything
    if (last_joint_state_msg_ == NULL || last_imu_msg_ == NULL) {
      continue;
    }

    // if not initialized and have ground truth: initialize the state based on
    // the ground truth data (mocap data)
    if (initialized == false && last_state_msg_ != NULL) {
      // initialize time
      last_time = ros::Time::now();

      // initialize X0,
      // initial guess for imu linear bias: -0.18, -0.07, -0.0
      // initial guess for imu angular bias: -0.00009, -0.00003, -0.00002
      X0 << (*last_state_msg_).body.pose.position.x,
          (*last_state_msg_).body.pose.position.y,
          (*last_state_msg_).body.pose.position.z,
          (*last_state_msg_).body.twist.linear.x,
          (*last_state_msg_).body.twist.linear.y,
          (*last_state_msg_).body.twist.linear.z,
          (*last_state_msg_).body.pose.orientation.w,
          (*last_state_msg_).body.pose.orientation.x,
          (*last_state_msg_).body.pose.orientation.y,
          (*last_state_msg_).body.pose.orientation.z,
          (*last_state_msg_).feet.feet[0].position.x,
          (*last_state_msg_).feet.feet[0].position.y,
          (*last_state_msg_).feet.feet[0].position.z,
          (*last_state_msg_).feet.feet[1].position.x,
          (*last_state_msg_).feet.feet[1].position.y,
          (*last_state_msg_).feet.feet[1].position.z,
          (*last_state_msg_).feet.feet[2].position.x,
          (*last_state_msg_).feet.feet[2].position.y,
          (*last_state_msg_).feet.feet[2].position.z,
          (*last_state_msg_).feet.feet[3].position.x,
          (*last_state_msg_).feet.feet[3].position.y,
          (*last_state_msg_).feet.feet[3].position.z, bias_x_, bias_y_, bias_z_,
          0, 0, 0;

      X = X0;
      last_X = X0;
      P = P0;
      initialized = true;
    } else if (initialized == false && last_state_msg_ == NULL) {
      // initialize time
      last_time = ros::Time::now();

      // initialize X0
      X0 << 0, 0, 0.3, 0, 0, 0, 1, 0, 0, 0,
          (*last_state_msg_).feet.feet[0].position.x,
          (*last_state_msg_).feet.feet[0].position.y,
          (*last_state_msg_).feet.feet[0].position.z,
          (*last_state_msg_).feet.feet[1].position.x,
          (*last_state_msg_).feet.feet[1].position.y,
          (*last_state_msg_).feet.feet[1].position.z,
          (*last_state_msg_).feet.feet[2].position.x,
          (*last_state_msg_).feet.feet[2].position.y,
          (*last_state_msg_).feet.feet[2].position.z,
          (*last_state_msg_).feet.feet[3].position.x,
          (*last_state_msg_).feet.feet[3].position.y,
          (*last_state_msg_).feet.feet[3].position.z, bias_x_, bias_y_, bias_z_,
          0, 0, 0;
      X = X0;
      last_X = X0;
      P = P0;
      initialized = true;
    }

    // Compute new state estimate
    quad_msgs::RobotState new_state_est = this->StepOnce();

    // Publish new state estimate
    state_estimate_pub_.publish(new_state_est);

    // Store new state estimate for next iteration
    last_state_est_ = new_state_est;

    // Enforce update rate
    r.sleep();
  }
}
