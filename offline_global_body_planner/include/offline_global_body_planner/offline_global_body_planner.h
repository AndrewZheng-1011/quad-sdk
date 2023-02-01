#ifndef OFFLINE_GLOBAL_BODY_PLANNER_H
#define OFFLINE_GLOBAL_BODY_PLANNER_H

#include <fstream>
#include <sstream>

#include <nav_msgs/Path.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "offline_global_body_planner/offline_global_body_plan.h"

using namespace offline_planning_utils;

//! Will be the official offline global body planner for legged robots
/*!
    OfflineGlobalBodyPlanner2 contains all the logic contained in the
   OfflineGlobalBodyPlanner node. Currently, it's main purpose is to generally
   take a trajectory of states and control inputs and publish it in a RobotPlan
   msg format.
*/
class OfflineGlobalBodyPlanner {
 public:
  /**
   * @brief Constructor for Offline Global Body Planner 2 class
   * @param nh Nodehandle
   */
  OfflineGlobalBodyPlanner(ros::NodeHandle nh);

  /**
   * @brief Reads in filePath trajectory and defines state and control
   * trajectory
   * @param stateFilePath csv filepath to state trajectory
   * @param controlFilePath csv filepath to control
   * @param numStates Dimension of states
   * @param numCtrl Dimension of control
   */
  void loadCSVData(
      std::string state_file_path, std::string ctrl_file_path, int num_states,
      int num_ctrl);  // may need plan object or simply state_sequence, etc.

  /**
   * @brief Set the start state to be used by the next planning call
   */
  void setStartState();

  /**
   * @brief Set the goal state to be used by the next planning call
   */
  void setGoalState();

  /**
   * @brief Call the planner
   * @return Whether calling plan went through or not
   */
  bool callPlanner();

  /**
   * @brief Publish FullState plan
   */
  void publishPlan();

  /**
   * @brief Callback function to handle new terrain map data
   * @param[in] msg the message containing map data
   */
  void terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg);

  /**
   * @brief Callback function to handle new robot state data
   * @param msg[in] msg the message containing robot state data
   */
  void robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg);

  /**
   * @brief Callback function to handle new goal state
   * @param msg the message containing the goal state
   */
  void goalStateCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

  /**
   * @brief Primary work function in class, called in node file for this
   * component
   */
  void spin();

 private:
  /**
   * @brief Set additional parameters to be viable for quad-sdk framework
   *
   */
  void setMiscPlans(double t0, double dt, std::vector<State> &state_sequence,
                    std::vector<GRF> &grf_plan, std::vector<double> &t_plan,
                    std::vector<int> &primitive_id_plan,
                    std::vector<double> &length_plan,
                    const PlannerConfig &planner_config);

  /**
   * @brief Triggers a reset event
   */
  void triggerReset();

  /**
   * @brief Wait until map and state messages have been received and processed
   * TODO(AZ): Do when get terrain info, etc.
   */
  void waitForData();

  /// Subscriber for terrain map messages
  ros::Subscriber terrain_map_sub_;

  /// Subscriber for robot state messages
  ros::Subscriber robot_state_sub_;

  /// Subscriber for body plan messages
  ros::Subscriber goal_state_sub_;  // Don't need this until online PF planner

  /// Publisher for body plan messages
  ros::Publisher body_plan_pub_;

  /// Publisher for discrete states in body plan messages
  ros::Publisher discrete_body_plan_pub_;

  /// Publisher for the planning tree
  ros::Publisher tree_pub_;  // Probably don't need this

  /// Topic name for terrain map (needed to ensure data has been received)
  std::string terrain_map_topic_;

  /// Topic name for robot state data (needed to ensure data has been received)
  std::string robot_state_topic_;

  /// Node Handle
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data
  double update_rate_;

  /// Handle for the map frame
  std::string map_frame_;

  /// Plan data (TODO(AZ): Future may want newest & current plan)
  OfflineGlobalBodyPlan plan_;

  /// Starting state for planner call
  FullState start_state_;

  /// Goal state for planner
  // TODO(AZ): Edit when want to change goal state mid-plan
  FullState goal_state_;

  /// Current robot state
  FullState robot_state_;

  /// goal_state_msg
  geometry_msgs::PointStamped::ConstPtr goal_state_msg_;

  /// Threshold of state error to trigger replanning
  double pos_error_threshold_;  // TODO(AZ): When using online planner

  /// Flag to determine if the planner needs to restart planning from
  /// the robot state
  bool restart_flag_;

  /// Sequences of discrete states in the plan
  std::vector<State> state_sequence_;

  /// Sequence of discrete grf in the plan
  std::vector<GRF> grf_sequence_;

  // TODO(AZ): Good to use if need these info for online planner
  /// Vector of cost instances in each planning call
  std::vector<double> cost_vector_;

  /// Vector of time instances of cost data for each planning call
  std::vector<double> cost_vector_times_;

  /// Vector of solve times for each planning call
  std::vector<double> solve_time_info_;

  /// Planner config
  PlannerConfig planner_config_;

  /// Boolean for whether replanning is allowed
  bool replanning_allowed_;  // TODO(AZ): use when online planner

  /// Timestep for interpolation
  double dt_;

  /// ID for status of planner
  int planner_status_;

  /// Index of activate plan from which to begin refinement
  // TODO(AZ): may need when online and some extra var for online
  // planning
  int start_index_;

  /// Timestamp for t=0 of global plan
  ros::Time global_plan_timestamp_;

  /// Time at which reset began
  ros::Time reset_time_;

  /// Delay after plan reset before publishing
  double reset_publish_delay_;

  /// Status of plan
  bool plan_status_;

  /// Published plan
  bool published_plan_;  // TODO(AZ): change variable when online and purpose

  /// Filepath for state trajectory
  std::string state_file_path_;

  /// Filepath for control trajectory
  std::string ctrl_file_path_;
};

#endif
