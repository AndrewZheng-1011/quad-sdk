#include "offline_global_body_planner/offline_global_body_planner.h"

OfflineGlobalBodyPlanner::OfflineGlobalBodyPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string body_plan_topic, discrete_body_plan_topic, body_plan_tree_topic,
      goal_state_topic;
  std::vector<double> goal_state_vec(2);  // Not being used

  quad_utils::loadROSParam(nh_, "topics/start_state", robot_state_topic_);
  quad_utils::loadROSParam(nh_, "topics/goal_state", goal_state_topic);
  quad_utils::loadROSParam(nh_, "/topics/terrain_map", terrain_map_topic_);
  quad_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan_discrete",
                           discrete_body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan_tree",
                           body_plan_tree_topic);
  quad_utils::loadROSParam(nh_, "/map_frame", map_frame_);
  quad_utils::loadROSParam(nh_, "/offline_global_body_planner/update_rate",
                           update_rate_);
  quad_utils::loadROSParam(nh_,
                           "/offline_global_body_planner/pos_error_threshold",
                           pos_error_threshold_);  // TODO(AZ): Use when online
  quad_utils::loadROSParam(nh_, "/offline_global_body_planner/startup_delay",
                           reset_publish_delay_);
  quad_utils::loadROSParam(nh_, "/offline_global_body_planner/replanning",
                           replanning_allowed_);
  quad_utils::loadROSParam(nh_, "/local_planner/timestep", dt_);
  quad_utils::loadROSParam(nh_, "/offline_global_body_planner/goal_state",
                           goal_state_vec);
  quad_utils::loadROSParam(nh_, "/offline_global_body_planner/state_file_path",
                           state_file_path_);
  quad_utils::loadROSParam(nh_, "/offline_global_body_planner/state_file_path",
                           ctrl_file_path_);

  // Setup pubs and subs
  terrain_map_sub_ =
      nh_.subscribe(terrain_map_topic_, 1,
                    &OfflineGlobalBodyPlanner::terrainMapCallback, this);
  robot_state_sub_ =
      nh_.subscribe(robot_state_topic_, 1,
                    &OfflineGlobalBodyPlanner::robotStateCallback, this);
  goal_state_sub_ = nh_.subscribe(
      goal_state_topic, 1, &OfflineGlobalBodyPlanner::goalStateCallback, this);

  body_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(body_plan_topic, 1);
  discrete_body_plan_pub_ =
      nh_.advertise<quad_msgs::RobotPlan>(discrete_body_plan_topic, 1);

  // Load planner config
  planner_config_.loadParamsFromServer(nh);

  // Fill in the goal state information
  goal_state_vec.resize(12, 0);
  vectorToFullState(goal_state_vec, goal_state_);

  start_index_ = 0;  // TODO(AZ): Use when online
  triggerReset();
}

void OfflineGlobalBodyPlanner::waitForData() {
  // Spin until terrain map message has been received and processed
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while ((shared_map == nullptr) && ros::ok()) {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>(
        terrain_map_topic_, nh_);
    ros::spinOnce();
  }

  boost::shared_ptr<quad_msgs::RobotState const> shared_robot_state;
  while ((shared_robot_state == nullptr) && ros::ok()) {
    shared_robot_state = ros::topic::waitForMessage<quad_msgs::RobotState>(
        robot_state_topic_, nh_);
    ros::spinOnce();
  }
  ROS_INFO("GBP has state and map information");
  reset_time_ = ros::Time::now();
}

void OfflineGlobalBodyPlanner::terrainMapCallback(
    const grid_map_msgs::GridMap::ConstPtr &msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  planner_config_.terrain.loadDataFromGridMap(map);  // Takes ~10ms
  planner_config_.terrain_grid_map = map;            // Takes ~0.1ms

  // Uodate the goal state of the planner
  goal_state_.pos[2] =
      planner_config_.h_nom + planner_config_.terrain.getGroundHeight(
                                  goal_state_.pos[0], goal_state_.pos[1]);
}

void OfflineGlobalBodyPlanner::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr &msg) {
  eigenToFullState(quad_utils::bodyStateMsgToEigen(msg->body), robot_state_);
  // ROS_WARN("In robotStateCallback");
  // printFullState(robot_state_);
}

void OfflineGlobalBodyPlanner::goalStateCallback(
    const geometry_msgs::PointStamped::ConstPtr &msg) {
  // TODO(AZ): Do when online planner...
}

void OfflineGlobalBodyPlanner::triggerReset() {
  // TODO(AZ): Edit more when online planner
  // planner_status_ = RESET; // Comment out for now.. When online, need to
  // refactor
  plan_.clear();
  reset_time_ = ros::Time::now();
  published_plan_ = false;
}

void OfflineGlobalBodyPlanner::setStartState() {
  // TODO(AZ): Edit when need for online planner
  if (!published_plan_) {
    start_state_ = robot_state_;
    // Comment out to print start state
    // ROS_WARN("Setting current robot state to start state");
    // printFullState(start_state_);
  }
}

void OfflineGlobalBodyPlanner::setGoalState() {}

bool OfflineGlobalBodyPlanner::callPlanner() {
  // Call planner by getting plan (loading from csv) or use other offline plan
  // class

  // In this case, get plan from CSV
  loadCSVData(state_file_path_, ctrl_file_path_, 4, 2);  // Load from params
  // TODO(AZ): Should just be 0.0 as start, might delete t0
  plan_status_ = plan_.loadPlanData(0.0, dt_, start_state_, state_sequence_,
                                    grf_sequence_, planner_config_);
}

void OfflineGlobalBodyPlanner::loadCSVData(std::string state_file_path,
                                           std::string ctrl_file_path,
                                           int num_states, int num_ctrl) {
  // Code intended to be upscale for 3D spatial case
  // Please generalize it if code is poorly written from that framework

  const int n = num_states;  // Dimension of states
  const int spatial_dim =
      n / 2;  // The dimension of the spatial space (e.g. 2D) (Half of the full
              // state dim) (Hard stop at 3)
  const int ctrl_dim = num_ctrl;  // Dimension of control (e.g. u_x & u_y)

  // File pointer
  std::ifstream stateInputFile;
  std::ifstream controlInputFile;

  std::vector<std::vector<double> > states_traj;
  std::vector<std::vector<double> > ctrls_traj;

  stateInputFile.open(state_file_path);
  controlInputFile.open(ctrl_file_path);

  if (stateInputFile.fail() || controlInputFile.fail()) {
    ROS_ERROR("Failed to open state or control trajectory csv file");
    ROS_ERROR("File paths: \n%s\n%s", state_file_path.c_str(),
              ctrl_file_path.c_str());
  }

  // Load data into stl vector of type State and GRF
  std::string state_line,
      ctrl_line;  // String object to read each line of state and ctrl (Can
                  // technically use just one line)
  State temp_state;
  GRF temp_ctrl;

  int counter = 0;
  while (std::getline(stateInputFile, state_line)) {
    counter++;
    // Read each line (i.e. All states at each index)
    // std::cout << "counter: " << counter << std::endl; // For debugging
    // purpose

    Eigen::Vector3d holder = Eigen::Vector3d::Zero();

    // Read each string entry in line and convert to double
    std::stringstream ss(state_line);
    std::string data;
    for (int i = 0; i < spatial_dim; i++) {
      std::getline(ss, data, ',');
      holder(i) = std::stod(data);
    }
    temp_state.pos = holder;
    holder = Eigen::Vector3d::Zero();  // Reset

    for (int j = 0; j < n - spatial_dim; j++) {
      std::getline(ss, data, ',');
      holder(j) = std::stod(data);
    }
    temp_state.vel = holder;
    state_sequence_.push_back(temp_state);
  }

  while (std::getline(controlInputFile, ctrl_line)) {
    counter++;
    // Read each line (i.e. All ctrl at each index)
    // std::cout << "counter: " << counter << std::endl; // For debugging
    // purpose

    Eigen::Vector3d holder = Eigen::Vector3d::Zero();

    // Read each string entry in line and convert to double
    std::stringstream ss(ctrl_line);
    std::string data;
    for (int i = 0; i < ctrl_dim; i++) {
      std::getline(ss, data, ',');
      holder(i) = std::stod(data);
    }
    temp_ctrl = holder;
    // std::cout << "ctrl: \n" << temp_ctrl << std::endl;
    grf_sequence_.push_back(temp_ctrl);
  }
}

void OfflineGlobalBodyPlanner::publishPlan() {
  // Declare the messages for interpolated body plan and discrete states,
  // initialize their headers

  // Condition for publishing plan:
  // 1) Plan not empty AND
  // 2) Reset publish delay has passed
  // 3) Plan status true (e.g. valid... need to change to enum or change name)
  if (plan_.isEmpty() ||
      ((ros::Time::now() - reset_time_).toSec() <= reset_publish_delay_) ||
      !plan_status_) {
    return;
  }

  quad_msgs::RobotPlan robot_plan_msg;
  quad_msgs::RobotPlan discrete_robot_plan_msg;

  robot_plan_msg.header.frame_id = map_frame_;
  robot_plan_msg.header.stamp =
      ros::Time::now();  // Initialize timestamp for msg

  // If first and only plan has not been published, set published timestamp for
  // plan
  if (!published_plan_) {
    // Initializing timestamp for your plan
    plan_.setPublishedTimestamp(ros::Time::now());
  }

  discrete_robot_plan_msg.header = robot_plan_msg.header;

  // Initialize the headers and types
  robot_plan_msg.global_plan_timestamp = plan_.getPublishedTimestamp();
  discrete_robot_plan_msg.global_plan_timestamp = plan_.getPublishedTimestamp();

  // Load the plan into the messages
  plan_.convertPlanToMsg(robot_plan_msg, discrete_robot_plan_msg);

  // Publish both messages
  body_plan_pub_.publish(robot_plan_msg);  // PUBLISH BODY PLAN TO MSG HERE...
                                           // ALREADY INTERPOLATED HERE IT SEEMS
  discrete_body_plan_pub_.publish(discrete_robot_plan_msg);

  if (!published_plan_) {
    ROS_WARN("First plan published, stamp = %f",
             robot_plan_msg.global_plan_timestamp.toSec());
    published_plan_ = true;
  }
}

void OfflineGlobalBodyPlanner::spin() {
  // Currently: Robot does not act well with when going in -180 or 180 yaw
  // direction Assuming it has to do w/ nmpc as wrap and unwrap works fine, but
  // not sure if there is that layer for nmpc

  ros::Rate r(update_rate_);

  // Wait until map and state data retrieved
  waitForData();
  ROS_INFO("Finished waiting for data...");

  // Load data
  setStartState();
  setGoalState();
  callPlanner();

  while (ros::ok()) {
    ros::spinOnce();

    // Publish the results
    publishPlan();
    r.sleep();
  }
}
