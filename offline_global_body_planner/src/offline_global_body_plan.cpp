#include "offline_global_body_planner/offline_global_body_plan.h"

/**
 * @brief Construct a new Offline Global Body Plan:: Offline Global Body Plan
 * object
 */
OfflineGlobalBodyPlan::OfflineGlobalBodyPlan() {
  // length_plan_.push_back(0);
}

void OfflineGlobalBodyPlan::clear() {
  t_plan_.clear();
  body_plan_.clear();
  state_sequence_.clear();
  grf_sequence_.clear();
  primitive_id_plan_.clear();
  length_plan_.clear();
}

bool OfflineGlobalBodyPlan::getNominalHeight(
    State &s, const PlannerConfig &planner_config) {
  // Check if state inside map
  if (!isInMap(s, planner_config)) {
    return false;
  }

  // std::cout << "planner_config.h_nom: " << planner_config.h_nom << std::endl;
  s.pos(2) =
      planner_config.h_nom + getTerrainZFilteredFromState(s, planner_config);
  return true;
}

void OfflineGlobalBodyPlan::addMG(GRF &grf,
                                  const PlannerConfig &planner_config) {
  grf(2) = planner_config.mass * planner_config.g;
}

void OfflineGlobalBodyPlan::addWalkID() {
  // TODO(AZ): take in param and add parameter for walk id
}

void OfflineGlobalBodyPlan::getDistToGoal() {
  // TODO(AZ): Far future when running online planner
}

void OfflineGlobalBodyPlan::printStateSequence(
    const std::vector<State> &state_sequence) {
  for (size_t i = 0; i != state_sequence.size(); i++) {
    std::cout << "Index: " << i << " ";
    printState(state_sequence.at(i));
  }
}

void OfflineGlobalBodyPlan::printGRFSequence(
    const std::vector<GRF> &grf_sequence) {
  for (size_t i = 0; i != grf_sequence.size(); i++) {
    std::cout << "Index: " << i << " ";
    printGRF(grf_sequence.at(i));
  }
}

void OfflineGlobalBodyPlan::printFullStatePlan(
    const std::vector<FullState> &full_state_plan) {
  for (size_t i = 0; i != full_state_plan.size(); i++) {
    std::cout << "Index: " << i << std::endl;
    printFullState(full_state_plan.at(i));
  }
}

void OfflineGlobalBodyPlan::addStateAndGRFToMsg(double t, int plan_index,
                                                const FullState &body_state,
                                                const GRF &grf,
                                                int primitive_id,
                                                quad_msgs::RobotPlan &msg) {
  // Represent each state as an Odometry message
  quad_msgs::RobotState state;
  quad_utils::updateStateHeaders(state,
                                 msg.global_plan_timestamp + ros::Duration(t),
                                 msg.header.frame_id, plan_index);

  // Load the data into the message
  state.body = quad_utils::eigenToBodyStateMsg(fullStateToEigen(body_state));

  quad_msgs::GRFArray grf_msg;
  geometry_msgs::Vector3 vector_msg;

  vector_msg.x = grf[0];
  vector_msg.y = grf[1];
  vector_msg.z = grf[2];
  geometry_msgs::Point point_msg;
  quad_utils::Eigen3ToPointMsg(body_state.pos, point_msg);

  grf_msg.header = state.header;
  grf_msg.vectors.push_back(vector_msg);
  grf_msg.points.push_back(point_msg);

  bool contact_state =
      (primitive_id !=
       2);  // TODO(AZ): Hardcoded for now... not sure what to do about this
  grf_msg.contact_states.push_back(contact_state);
  msg.states.push_back(state);
  msg.grfs.push_back(grf_msg);
  msg.plan_indices.push_back(plan_index);
  msg.primitive_ids.push_back(primitive_id);
}

void OfflineGlobalBodyPlan::convertPlanToMsg(
    quad_msgs::RobotPlan &robot_plan_msg,
    quad_msgs::RobotPlan &discrete_robot_plan_msg) {
  // Get body plan size
  int num_discrete_pts = t_plan_.size();

  // Loop through body plan and add to message
  for (std::size_t i = 0; i != num_discrete_pts; i++) {
    addStateAndGRFToMsg(t_plan_[i], i, body_plan_[i], grf_sequence_[i],
                        primitive_id_plan_[i], robot_plan_msg);
  }
  // Loop through discrete states and add to message

  for (std::size_t i = 0; i != state_sequence_.size(); i++) {
    FullState full_discrete_state =
        stateToFullState(state_sequence_[i], 0, 0, 0, 0, 0, 0);
    addStateAndGRFToMsg(0.0, 0, full_discrete_state, grf_sequence_[i],
                        primitive_id_plan_[i], discrete_robot_plan_msg);
  }

  if (robot_plan_msg.states.size() != robot_plan_msg.grfs.size()) {
    throw std::runtime_error(
        "Mismatch between number of states and wrenches, something is wrong");
  }
}

bool OfflineGlobalBodyPlan::loadPlanData(double t0, double dt,
                                         const FullState &start_state,
                                         std::vector<State> &state_sequence,
                                         std::vector<GRF> &grf_sequence,
                                         const PlannerConfig &planner_config) {
  // TODO (AZ): Work on interpolating if dt of plan and dt of offline plan do
  // not match up Check validity of dt
  if (dt <= 0) {
    ROS_ERROR("Invalid dt = %0.2f. dt must be greater than 0.", dt);
  }

  int num_discrete_pts = state_sequence.size();  // state trajectory size
  double plan_length = 0;                        // Initialize plan length
  plan_loaded_ = true;  // For now, set plan to be loaded as true here unless
                        // otherwise... weak and not robust

  // Configure plans to quad-sdk framework
  for (int i = 0; i < num_discrete_pts; i++) {
    // Add 3rd dim to state & vel (set pos to nominal height at pos of terrain &
    // vel to 0)
    if (!getNominalHeight(state_sequence.at(i), planner_config)) {
      // ROS_WARN("Global plan is off the map!!");
      plan_loaded_ = false;
    }

    // Set v_z to be aligned with surf norm, but also in proportion to planar
    // velocity
    state_sequence.at(i).vel[2] =
        getDzFromState(state_sequence.at(i), planner_config);

    // Add 3rd dim to inputs (i.e. fixed mg)
    addMG(grf_sequence.at(i), planner_config);

    // Fill in time data
    t_plan_.push_back(t0 + i * dt);

    // Fill in hardcoded connect primitive id (e.g. 0 == CONNECT == STANCE)
    primitive_id_plan_.push_back(0);

    // Fill in length of plan at each index
    if (i == 0) {
      length_plan_.push_back(plan_length);
    } else {
      plan_length = plan_length + poseDistance(state_sequence.at(i - 1),
                                               state_sequence.at(i));
      length_plan_.push_back(plan_length);
    }
  }

  // Lift from reduced plan into full body plan
  addFullStates(start_state, state_sequence, dt, body_plan_,
                planner_config);  // TODO(AZ): ensure works for any dt | works
                                  // for 0.01 atm

  // Transfer modified sequence to plan
  state_sequence_ = state_sequence;
  grf_sequence_ = grf_sequence;

  // Not best code design
  if (!plan_loaded_) {
    ROS_ERROR("Global plan is off the map!!!");
    return false;
  }
  return true;
}
