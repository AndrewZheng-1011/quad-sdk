#include "spirit_utils/rviz_interface.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

RVizInterface::RVizInterface(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string body_plan_topic, body_plan_viz_topic, body_wrench_plan_viz_topic, discrete_body_plan_topic, discrete_body_plan_viz_topic,
    footstep_plan_topic, footstep_plan_viz_topic, swing_leg_plan_topic, state_estimate_topic, ground_truth_state_topic, joint_states_viz_topic;

  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/discrete_body_plan", discrete_body_plan_topic, "/discrete_body_plan");
  nh.param<std::string>("topics/footstep_plan", footstep_plan_topic, "/footstep_plan");
  nh.param<std::string>("topics/swing_leg_plan", swing_leg_plan_topic, "/swing_leg_plan");
  nh.param<std::string>("topics/state/estimate", state_estimate_topic, "/state/estimate");
  nh.param<std::string>("topics/state/ground_truth", ground_truth_state_topic, "/state/ground_truth");
  
  nh.param<std::string>("topics/visualization/body_plan", body_plan_viz_topic, "/visualization/body_plan");
  nh.param<std::string>("topics/visualization/body_wrench_plan", body_wrench_plan_viz_topic, "/visualization/body_wrench_plan");
  nh.param<std::string>("topics/visualization/discrete_body_plan", discrete_body_plan_viz_topic, "/visualization/discrete_body_plan");
  nh.param<std::string>("topics/visualization/footstep_plan", footstep_plan_viz_topic, "/visualization/footstep_plan");
  nh.param<std::string>("topics/visualization/joint_states", joint_states_viz_topic, "/visualization/joint_states");


  nh.param<std::string>("map_frame",map_frame_,"map");
  nh.param<double>("visualization/update_rate", update_rate_, 10); // add a param for your package instead of using the estimator one

  // Setup pubs and subs
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,&RVizInterface::bodyPlanCallback, this);
  discrete_body_plan_sub_ = nh_.subscribe(discrete_body_plan_topic,1,&RVizInterface::discreteBodyPlanCallback, this);
  footstep_plan_sub_ = nh_.subscribe(footstep_plan_topic,1,&RVizInterface::footstepPlanCallback, this);
  swing_leg_plan_sub_ = nh_.subscribe(swing_leg_plan_topic,1,&RVizInterface::swingLegPlanCallback, this);
  state_estimate_sub_ = nh_.subscribe(state_estimate_topic,1,&RVizInterface::stateEstimateCallback, this);
  ground_truth_state_sub_ = nh_.subscribe(ground_truth_state_topic,1,&RVizInterface::groundTruthStateCallback, this);
  body_plan_viz_pub_ = nh_.advertise<nav_msgs::Path>(body_plan_viz_topic,1);
  body_wrench_plan_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(body_wrench_plan_viz_topic,1);
  discrete_body_plan_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(discrete_body_plan_viz_topic,1);
  footstep_plan_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(footstep_plan_viz_topic,1);
  joint_states_viz_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_states_viz_topic,1);

  std::string swing_leg_0_plan_viz_topic, swing_leg_1_plan_viz_topic, swing_leg_2_plan_viz_topic, swing_leg_3_plan_viz_topic;

  nh.param<std::string>("topics/visualization/swing_leg_0_plan", swing_leg_0_plan_viz_topic, "/visualization/swing_leg_0_plan");
  nh.param<std::string>("topics/visualization/swing_leg_1_plan", swing_leg_1_plan_viz_topic, "/visualization/swing_leg_1_plan");
  nh.param<std::string>("topics/visualization/swing_leg_2_plan", swing_leg_2_plan_viz_topic, "/visualization/swing_leg_2_plan");
  nh.param<std::string>("topics/visualization/swing_leg_3_plan", swing_leg_3_plan_viz_topic, "/visualization/swing_leg_3_plan");

  swing_leg_0_plan_viz_pub_ = nh_.advertise<nav_msgs::Path>(swing_leg_0_plan_viz_topic,1);
  swing_leg_1_plan_viz_pub_ = nh_.advertise<nav_msgs::Path>(swing_leg_1_plan_viz_topic,1);
  swing_leg_2_plan_viz_pub_ = nh_.advertise<nav_msgs::Path>(swing_leg_2_plan_viz_topic,1);
  swing_leg_3_plan_viz_pub_ = nh_.advertise<nav_msgs::Path>(swing_leg_3_plan_viz_topic,1);

}

void RVizInterface::bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {

  // Initialize Path message to visualize body plan
  nav_msgs::Path body_plan_viz;
  body_plan_viz.header = msg->header;

  // Loop through the BodyPlan message to get the state info and add to private vector
  int length = msg->states.size();
  for (int i=0; i < length; i++) {

    // Load in the pose data directly from the Odometry message
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->states[i].header;
    pose_stamped.pose = msg->states[i].pose.pose;

    // Add to the path message
    body_plan_viz.poses.push_back(pose_stamped);
  }

  // Publish the full path
  body_plan_viz_pub_.publish(body_plan_viz);


  // Construct MarkerArray and Marker message for GRFs
  visualization_msgs::MarkerArray wrenches_msg;
  visualization_msgs::Marker marker;

  // Initialize the headers and types
  marker.header = msg->header;
  marker.type = visualization_msgs::Marker::ARROW;

  // Define the shape of the discrete states
  double arrow_diameter = 0.01;
  marker.scale.x = arrow_diameter;
  marker.scale.y = 2*arrow_diameter;
  marker.color.g = 0.733f;
  marker.pose.orientation.w = 1.0;

  for (int i=0; i < length; i++) {
    
    // Reset the marker message
    marker.points.clear();
    marker.color.a = 1.0;
    marker.id = i;

    // Define point messages for the base and tip of each GRF arrow
    geometry_msgs::Point p_base, p_tip;
    p_base = msg->states[i].pose.pose.position;

    /// Define the endpoint of the GRF arrow
    double grf_length_scale = 0.001;
    p_tip.x = p_base.x + grf_length_scale*msg->wrenches[i].force.x;
    p_tip.y = p_base.y + grf_length_scale*msg->wrenches[i].force.y;
    p_tip.z = p_base.z + grf_length_scale*msg->wrenches[i].force.z;

    // if GRF = 0, set alpha to zero
    if (msg->wrenches[i].force.z < 1e-4) {
      marker.color.a = 0.0;
    }

    // Add the points to the marker and add the marker to the array
    marker.points.push_back(p_base);
    marker.points.push_back(p_tip);
    wrenches_msg.markers.push_back(marker);
  }

  // Publish grfs
  body_wrench_plan_viz_pub_.publish(wrenches_msg);
}

void RVizInterface::discreteBodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {

  // Construct Marker message
  visualization_msgs::Marker discrete_body_plan;

  // Initialize the headers and types
  discrete_body_plan.header = msg->header;
  discrete_body_plan.id = 0;
  discrete_body_plan.type = visualization_msgs::Marker::POINTS;

  // Define the shape of the discrete states
  double scale = 0.2;
  discrete_body_plan.scale.x = scale;
  discrete_body_plan.scale.y = scale;
  discrete_body_plan.color.r = 0.733f;
  discrete_body_plan.color.a = 1.0;

  // Loop through the discrete states
  int length = msg->states.size();
  for (int i=0; i < length; i++) {
    geometry_msgs::Point p;
    p.x = msg->states[i].pose.pose.position.x;
    p.y = msg->states[i].pose.pose.position.y;
    p.z = msg->states[i].pose.pose.position.z;
    discrete_body_plan.points.push_back(p);
  }

  // Publish both interpolated body plan and discrete states
  discrete_body_plan_viz_pub_.publish(discrete_body_plan);
}

void RVizInterface::footstepPlanCallback(const spirit_msgs::FootstepPlan::ConstPtr& msg) {

  // Initialize Marker message to visualize footstep plan as points
  visualization_msgs::Marker points;
  points.header = msg->header;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;

  // Loop through each foot
  int num_feet = msg->feet.size();
  for (int i=0;i<num_feet; ++i) {

    // Loop through footstep in the plan
    int num_steps = msg->feet[i].steps.size();
    for (int j=0;j<num_steps; ++j) {

      // Create point message from FootstepPlan message, adjust height
      geometry_msgs::Point p;
      p = msg->feet[i].steps[j].position;

      // Set the color properties of each marker (green for front feet, blue for back)
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      if (i == 0 || i == 2) {
        color.g = 1.0f;
      } else {
        color.b = 1.0f;
      }

      // Add to the Marker message
      points.colors.push_back(color);
      points.points.push_back(p);
    }


  }

  // Publish the full marker array
  footstep_plan_viz_pub_.publish(points);
}

void RVizInterface::swingLegPlanCallback(const spirit_msgs::SwingLegPlan::ConstPtr& msg) {

  swing_leg_0_plan_viz_pub_.publish(msg->legs[0]);
  swing_leg_1_plan_viz_pub_.publish(msg->legs[1]);
  swing_leg_2_plan_viz_pub_.publish(msg->legs[2]);
  swing_leg_3_plan_viz_pub_.publish(msg->legs[3]);

}

void RVizInterface::stateEstimateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {

  // // Make a transform message for the body, populate with state estimate data, and publish
  // geometry_msgs::TransformStamped transformStamped;
  // transformStamped.header = msg->header;
  // transformStamped.header.frame_id = map_frame_;
  // transformStamped.child_frame_id = "dummy";
  // transformStamped.transform.translation.x = msg->body.pose.pose.position.x;
  // transformStamped.transform.translation.y = msg->body.pose.pose.position.y;
  // transformStamped.transform.translation.z = msg->body.pose.pose.position.z;
  // transformStamped.transform.rotation = msg->body.pose.pose.orientation;
  // estimate_base_tf_br_.sendTransform(transformStamped);

  // // Copy the joint portion of the state estimate message to a new message
  // sensor_msgs::JointState joint_msg;
  // joint_msg = msg->joints;

  // // Set the header to the main header of the state estimate message and publish
  // joint_msg.header = msg->header;
  // joint_states_viz_pub_.publish(joint_msg);
}


void RVizInterface::groundTruthStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {

  // Make a transform message for the body, populate with state estimate data, and publish
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header = msg->header;
  transformStamped.header.frame_id = map_frame_;
  transformStamped.child_frame_id = "dummy";
  transformStamped.transform.translation.x = msg->body.pose.pose.position.x;
  transformStamped.transform.translation.y = msg->body.pose.pose.position.y;
  transformStamped.transform.translation.z = msg->body.pose.pose.position.z;
  transformStamped.transform.rotation = msg->body.pose.pose.orientation;
  ground_truth_base_tf_br_.sendTransform(transformStamped);

  // Copy the joint portion of the state estimate message to a new message
  sensor_msgs::JointState joint_msg;
  joint_msg = msg->joints;

  // Set the header to the main header of the state estimate message and publish
  joint_msg.header = msg->header;
  joint_states_viz_pub_.publish(joint_msg);
}

void RVizInterface::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce();
    
    // Enforce update rate
    r.sleep();
  }
}