#ifndef OFFLINE_PLANNING_UTILS_H
#define OFFLINE_PLANNING_UTILS_H

#include <math.h>
#include <quad_utils/fast_terrain_map.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>

// Uncomment to add features
// #define DEBUG_INVALID_STATE 1

namespace offline_planning_utils {

/**
 * @brief Planner Configuration
 *
 */
struct PlannerConfig {
  FastTerrainMap terrain;              // Terrain in FastTerrainMap format
  grid_map::GridMap terrain_grid_map;  // Terrain in grid_map format

  // Define kinematic constraint paramters
  double h_max;  // Maximum hiehgt of a leg base, m (prob don't need)
  double h_min;  // Minimum ground clearance of body corners, m
  double h_nom;  // Nominal ground clearance of body, m
  double v_max;  // Maximum robot velocity, m/s
  double v_nom;  // Nominal robot velocity, m/s

  // Define dynamic parameters
  double mass;  // Robot mass, kg
  double g;     // Gravity constant, m/s^2
  double mu;    // TODO(AZ): need when online planner

  // Define planning parameters
  double dt;
  Eigen::Vector3d g_vec;

  // Define robot params and declare points used for validity checking (not sure
  // needed here yet)
  double robot_l;  // Length of robot body, m
  double robot_w;  // Width of robot body, m
  double robot_h;  // Vertical distance between leg base and bottom of robot, m

  void loadParamsFromServer(ros::NodeHandle nh) {
    // Load robot parameters
    // TODO(AZ): Find way to load these params nicely that are loaded from robot
    // yaml
    quad_utils::loadROSParam(nh, "offline_global_body_planner/h_max", h_max);
    quad_utils::loadROSParam(nh, "offline_global_body_planner/h_min", h_min);
    quad_utils::loadROSParam(nh, "offline_global_body_planner/h_nom", h_nom);
    quad_utils::loadROSParam(nh, "offline_global_body_planner/v_max", v_max);
    quad_utils::loadROSParam(nh, "offline_global_body_planner/v_nom", v_nom);
    quad_utils::loadROSParam(nh, "offline_global_body_planner/robot_l",
                             robot_l);
    quad_utils::loadROSParam(nh, "offline_global_body_planner/robot_w",
                             robot_w);
    quad_utils::loadROSParam(nh, "offline_global_body_planner/robot_h",
                             robot_h);
    /*
    std::cout << "List of parameter names: " << std::endl;

    std::vector<std::string> keys;
    nh.getParamNames(keys);
    for(std::size_t i = 0; i < keys.size(); ++i) {
        std::cout << keys[i] << "\n";
    }
    */
    // std::cout << "h_nom from loading planner_config: " << h_nom << std::endl;
    // std::cout << "h_max: " << h_max << std::endl;

    quad_utils::loadROSParam(nh, "offline_global_body_planner/mass", mass);
    // quad_utils::loadROSParam(nh, "global_body_planner/grf_min", grf_min);
    // quad_utils::loadROSParam(nh, "global_body_planner/grf_max", grf_max);
    // quad_utils::loadROSParam(nh,
    //                         "/global_body_planner/traversability_threshold",
    //                         traversability_threshold);

    // Load global parameters
    quad_utils::loadROSParam(nh, "/offline_global_body_planner/g", g);
    quad_utils::loadROSParam(nh, "/offline_global_body_planner/mu", mu);
    // quad_utils::loadROSParam(nh, "/global_body_planner/t_s_min", t_s_min);
    // quad_utils::loadROSParam(nh, "/global_body_planner/t_s_max", t_s_max);
    // quad_utils::loadROSParam(nh, "/global_body_planner/dz0_min", dz0_min);
    // quad_utils::loadROSParam(nh, "/global_body_planner/dz0_max", dz0_max);
    quad_utils::loadROSParam(nh, "/global_body_planner/dt",
                             dt);  // Might not need this... dt is for
                                   // resolution of kinematic feasibility check
    // quad_utils::loadROSParam(nh, "/global_body_planner/backup_ratio",
    //                         backup_ratio); // TODO(AZ): Might need for online
    //                         planner
    // quad_utils::loadROSParam(nh,
    // "/global_body_planner/trapped_buffer_factor",
    //                         trapped_buffer_factor);
    // quad_utils::loadROSParam(nh, "/global_body_planner/num_leap_samples",
    //                         num_leap_samples);
    // quad_utils::loadROSParam(nh, "/global_body_planner/max_planning_time",
    //                         max_planning_time);

    // Load the scalar parameters into Eigen vectors
    // loadEigenVectorsFromParams(); // Reachibility test and collision test not
    // needed
    g_vec << 0, 0, -g;
  }
};

enum ExitFlag { INVALID_START_GOAL_EQUAL };

/**
 * @brief Define state with Eigen data
 *
 */
struct State {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
};

/**
 * @brief Define full state with Eigen data
 *
 */
struct FullState {
  Eigen::Vector3d pos;      // Position
  Eigen::Vector3d vel;      // Velocity
  Eigen::Vector3d ang;      // Linear Velocity
  Eigen::Vector3d ang_vel;  // Angular velocity
};

/// Ground reaction force
typedef Eigen::Vector3d GRF;

/**
 * @brief Append FullState to State and FullState arrays
 * @param start_state FullState to be appended
 * @param reduced_plan Sequence of States to be appended upon
 * @param dt Time resolution
 * @param full_plan The sequence of FullStates being appended on
 * @param planner_config Configuration parameters
 */
void addFullStates(const FullState &start_state,
                   const std::vector<State> &reduced_plan, double dt,
                   std::vector<FullState> &full_plan,
                   const PlannerConfig &planner_config);

/**
 * @brief Convert reduced State to FullState, adding body oroientation and
 * angular speed
 * @param state State
 * @param roll Roll
 * @param pitch Pitch
 * @param yaw Yaw
 * @param roll_rate Rate of change of roll
 * @param pitch_rate Rate of change of pitch
 * @param yaw_rate Rate of change of yaw
 * @return FullState
 */
FullState stateToFullState(const State &state, double roll, double pitch,
                           double yaw, double roll_rate, double pitch_rate,
                           double yaw_rate);

/**
 * @brief Convert fullstate to state
 * @param full_state FullState
 * @return State
 */
State fullStateToState(const FullState &full_state);

/**
 * @brief Reformat Eigen vector to FullState
 * @param[in] s_eig Eigen vector
 * @param[out] s FullState obtained from Eigen vector
 */
void eigenToFullState(const Eigen::VectorXd &s_eig, FullState &s);

/**
 * @brief Reformat FullState to Eigen Vector
 */
Eigen::VectorXd fullStateToEigen(const FullState &s);

/**
 * @brief Reformat STL vector to FullState
 * @param[in] v STL vector
 * @param[out] s FullState obtained from STL vector
 */
void vectorToFullState(const std::vector<double> &v, FullState &s);

/**
 * @brief Print State
 * @param s State
 */
void printState(const State &s);

/**
 * @brief Print GRF
 * @param grf GRF
 */
void printGRF(const GRF &grf);

inline double poseDistance(const State &s1, const State &s2) {
  return (s1.pos - s2.pos).norm();
}

/**
 * @brief Print FullState
 * @param s FullState
 */
void printFullState(const FullState &s);

/**
 * @brief Inline function to get the terrain z filtered at a point from a
 * position
 * @param pos Position
 * @param planner_config Configuration parameters
 * @return double Height of terrain at a point
 */
inline double getTerrainZFiltered(const Eigen::Vector3d &pos,
                                  const PlannerConfig &planner_config) {
  // Uncomment to use grid_map
  // return planner_config.terrain_grid_map.atPosition("z_smooth",
  // pos.head<2>(),
  //                                             INTER_TYPE);
  return (planner_config.terrain.getGroundHeightFiltered(pos[0], pos[1]));
}

/**
 * @brief Inline function to get the filtered terrain height at a point from
 * state
 * @param pos State position
 * @param planner_config Configuration parameters
 */
inline double getTerrainZFilteredFromState(
    const State &s, const PlannerConfig &planner_config) {
  return getTerrainZFiltered(s.pos, planner_config);
}

inline Eigen::Vector3d getSurfaceNormalFiltered(
    const State &s, const PlannerConfig &planner_config) {
  // Uncomment to use grid_map
  // Eigen::Vector3d surf_norm;
  // surf_norm.x() = planner_config.terrain_grid_map.atPosition(
  //     "normal_vectors_x", s.pos.head<2>(), INTER_TYPE);
  // surf_norm.y() = planner_config.terrain_grid_map.atPosition(
  //     "normal_vectors_y", s.pos.head<2>(), INTER_TYPE);
  // surf_norm.z() = planner_config.terrain_grid_map.atPosition(
  //     "normal_vectors_z", s.pos.head<2>(), INTER_TYPE);
  // return surf_norm;
  return planner_config.terrain.getSurfaceNormalFilteredEigen(s.pos[0],
                                                              s.pos[1]);
}

/**
 * @brief Get the body pitch from state
 * @param s State
 * @param planner_config Configuration parameters
 * @return Pitch of current state
 */
double getPitchFromState(const State &s, const PlannerConfig &planner_config);

/**
 * @brief Align lateral velocity along surface normal of filtered map
 * @param s State
 * @param planner_config Configuration parameters
 * @return delta z
 */
double getDzFromState(const State &s, const PlannerConfig &planner_config);

/**
 * @brief Inline function to check if pos is inside map range or not
 * @param pos State pos in Eigen Vector form
 * @param planner_config Configuration parameter
 * @return Whether State pos is inside map
 */
inline bool isInMap(const Eigen::Vector3d &pos,
                    const PlannerConfig &planner_config) {
  return planner_config.terrain_grid_map.isInside(pos.head<2>());
}

/**
 * @brief Inline function to check if State pos is inside map range or not
 * @param s State
 * @param planner_config Configuration parameters
 * @return true State pos is inside map
 * @return false State pos is not inside map
 */
inline bool isInMap(const State &s, const PlannerConfig &planner_config) {
  return isInMap(s.pos, planner_config);
}

/**
 * @brief
 * @param s
 * @param planner_config
 * @param phase
 * @return true
 * @return false
 */
bool isValidState(const State &s, const PlannerConfig &planner_config,
                  int phase);  // TODO(AZ): Integrate quad-sdk isValidState when
                               // necessary to do all checks

}  // namespace offline_planning_utils

#endif
