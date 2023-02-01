#ifndef OFFLINE_GLOBAL_BODY_PLAN_H
#define OFFLINE_GLOBAL_BODY_PLAN_H

#include <quad_utils/ros_utils.h>

#include "offline_global_body_planner/offline_planning_utils.h"

using namespace offline_planning_utils;

//! A class to contain offline global plan data along with helper functions
/*!
    This class reads a 2D planner from a CSV file and modifies to a "3D planner"
    for quadruped locomotion. Helper function are useful for transforming 2D
    planner to 3D.
*/
class OfflineGlobalBodyPlan {
 public:
  /**
   * @brief Constructor for OfflineGlobalBodyPlan
   * @return Constructed object of type OfflineGlobalBodyPlan
   */
  OfflineGlobalBodyPlan();

  /**
   * @brief Clears all data and reset the length plan
   */
  void clear();

  /**
   * @brief Checks if the plan is empty
   * @return Plan emptiness
   */
  inline bool isEmpty() const { return t_plan_.empty(); }

  /**
   * @brief Get the position of quadruped given height and location w/in terrain
   * @param pos State position
   * @param planner_config Planner configuration
   * @return Nominal height loaded into State variable
   */
  bool getNominalHeight(State &s, const PlannerConfig &planner_config);

  /**
   * @brief Add mg to GRF due to planar planner
   * @param grf Ground Reaction force added with fixed z component
   */
  void addMG(GRF &grf, const PlannerConfig &planner_config);

  /**
   * @brief Add fixed primitive ID into plan (Remove in future potentially - AZ)
   */
  void addWalkID();

  /**
   * @brief Get dist to goal from control and state trajectory
   */
  void getDistToGoal();  // Probably won't need this until far future

  /**
   * @brief Print state_sequence
   * @param state_sequence Sequence of state
   */
  void printStateSequence(const std::vector<State> &state_sequence);

  /**
   * @brief Print grf_sequence
   * @param grf_sequence Sequence of action
   */
  void printGRFSequence(const std::vector<GRF> &grf_sequence);

  /**
   * @brief Print FullState plan
   * @param full_state_plan FullState plan
   */
  void printFullStatePlan(const std::vector<FullState> &full_state_plan);

  /**
   * @brief Get the Length Of Plan
   * @return double The distance [m] of the plan
   */
  inline double getLengthOfPlan() const {
    if (length_plan_.empty()) {
      ROS_WARN("Tried to retrieve length of plan when empty");
      return 0.0;
    }
    return length_plan_.back();
  }

  /**
   * @brief Set the timestamp at which the plan was computed (unique)
   * @param timestamp Timestamp at which plan returned
   */
  inline void setComputedTimestamp(ros::Time timestamp) {
    computed_timestamp_ = timestamp;
  }

  /**
   * @brief Set the timestamp at which the plan was published
   * @param timestamp Timestamp at which plan was published
   */
  inline void setPublishedTimestamp(ros::Time timestamp) {
    published_timestamp_ = timestamp;
  }

  /**
   * @brief Get the Computed Timestamp
   * @return ros::Time Timestamp at which plan computed
   */
  inline ros::Time getComputedTimestamp() const { return computed_timestamp_; }

  /**
   * @brief Get the Published Timestamp
   * @return ros::Time Timestamp at which a plan was published
   */
  inline ros::Time getPublishedTimestamp() const {
    return published_timestamp_;
  }

  /**
   * @brief Loads all body plan into a RobotPlan msg
   * @param robot_plan_msg Robot plan message
   * @param discrete_robot_plan_msg Discrete robot plan message
   */
  void convertPlanToMsg(quad_msgs::RobotPlan &robot_plan_msg,
                        quad_msgs::RobotPlan &discrete_robot_plan_msg);

  /**
   * @brief Load plan by adding additional parameters to be viable for quad-sdk
   * framework
   * @param t0 Start time of plan data
   * @param dt Timestep of plan
   * @param start_state Start state of robot
   * @param state_sequence Discrete state sequences to be filled
   * @param grf_plan Discrete control sequence to be filled
   * @param planner_config Planning configuration parameters
   */
  bool loadPlanData(double t0, double dt, const FullState &start_state,
                    std::vector<State> &state_sequence,
                    std::vector<GRF> &grf_plan,
                    const PlannerConfig &planner_config);

 private:
  /**
   * @brief
   * @param t Time at which this data occurs
   * @param plan_index Index in the plan for which this data will be inserted
   * @param body_state Body state data
   * @param grf GRF data
   * @param primitive_id ID for stance (LEAP), FLIGHT, or LAND (0 for stance)
   * @param[out] msg Robot plan message with data added
   */
  void addStateAndGRFToMsg(double t, int plan_index,
                           const FullState &body_state, const GRF &grf,
                           int primitive_id, quad_msgs::RobotPlan &msg);

  /// Time stamp for when plan was computed/loaded in this case (unique)
  ros::Time computed_timestamp_;

  /// Time stamp for when plan was published (not unique)
  ros::Time published_timestamp_;

  /// Std vector containing the robot body plan
  std::vector<FullState> body_plan_;

  /// Std vector containing the discrete states in the sequence (not quite a
  /// plan yet.. feel free to name better)
  std::vector<State> state_sequence_;

  /// Std vector containing the GRF in the plan
  std::vector<GRF> grf_sequence_;

  /// Std vector containing primitive id plan
  std::vector<int> primitive_id_plan_;

  /// Std vector containing time data
  std::vector<double> t_plan_;

  /// Std vector containing cimulative path length at each index of the plan
  std::vector<double> length_plan_;

  /// Plan successfully loaded into plan class
  bool plan_loaded_;
};

#endif  // OFFLINE_GLOBAL_BODY_PLAN_H
