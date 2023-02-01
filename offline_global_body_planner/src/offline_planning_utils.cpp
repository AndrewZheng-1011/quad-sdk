#include "offline_global_body_planner/offline_planning_utils.h"

namespace offline_planning_utils {

void addFullStates(const FullState &start_state,
                   const std::vector<State> &reduced_plan, double dt,
                   std::vector<FullState> &full_plan,
                   const PlannerConfig &planner_config) {
  // TODO(AZ): Refactor and integrate planner_config **
  int num_discrete_pts = reduced_plan.size();

  // Set roll and roll rate to zero
  double roll = 0;
  double roll_rate = 0;

  // Declare variable for yaw
  std::vector<double> z(num_discrete_pts);
  std::vector<double> filtered_z(num_discrete_pts);
  std::vector<double> z_rate(num_discrete_pts);
  std::vector<double> filtered_z_rate(num_discrete_pts);
  std::vector<double> pitch(num_discrete_pts);
  std::vector<double> pitch_rate(num_discrete_pts);
  std::vector<double> filtered_pitch_rate(num_discrete_pts);
  std::vector<double> wrapped_yaw(num_discrete_pts);
  std::vector<double> unwrapped_yaw(num_discrete_pts);
  std::vector<double> filtered_yaw(num_discrete_pts);
  std::vector<double> yaw_rate(num_discrete_pts);
  std::vector<double> filtered_yaw_rate(num_discrete_pts);

  // Enforce that yaw and pitch match current state, let the filter smooth
  // things out
  z[0] = start_state.pos[2];
  pitch[0] = start_state.ang[1];
  wrapped_yaw[0] = start_state.ang[2];

  // Calculate following wrapped yaw to align with heading
  for (int i = 1; i < num_discrete_pts; i++) {
    State body_state = reduced_plan[i];
    wrapped_yaw[i] = atan2(body_state.vel[1], body_state.vel[0]);
  }

  // Unwrap yaw for filtering / calculating rate (need for calculating rate atm)
  unwrapped_yaw = math_utils::unwrap(wrapped_yaw);

  /*
  for (int i = 0; i < unwrapped_yaw.size(); i++) {
      std::cout << "unwrapped_yaw[" << i << "]: " << unwrapped_yaw[i] <<
  std::endl;
  }
  */

  // Compute pitch to align with the terrain (may want to add first order filter
  // on init state)
  double gamma = 1.0;  // Original: 0.98
  for (int i = 1; i < num_discrete_pts; i++) {
    double weight = pow(gamma, i);
    State body_state = reduced_plan[i];
    z[i] =
        body_state.pos[2];  // weight * z[0] + (1 - weight) * body_state.pos[2];
    pitch[i] = getPitchFromState(
        body_state,
        planner_config);  // weight * pitch[0] +
                          // (1 - weight) * getPitchFromState(body_state,
                          // planner_config); // TODO(AZ): Use when need filter
                          // unwrapped_yaw[i] =
    //    weight * unwrapped_yaw[0] + (1 - weight) * unwrapped_yaw[i]; //
    //    Uncomment to apply filtering
  }

  // Hold penultimate yaw value to avoid singularity issues at terminal state
  // (often zero velocity)
  unwrapped_yaw[num_discrete_pts - 1] = unwrapped_yaw[num_discrete_pts - 2];

  // Filter yaw and compute its derivative via central difference method)
  int window_size = 1;  // Original value: 25
  filtered_yaw = math_utils::movingAverageFilter(unwrapped_yaw, window_size);
  yaw_rate = math_utils::centralDiff(filtered_yaw, dt);
  filtered_yaw_rate = math_utils::movingAverageFilter(yaw_rate, window_size);
  pitch_rate = math_utils::centralDiff(pitch, dt);
  filtered_pitch_rate =
      math_utils::movingAverageFilter(pitch_rate, window_size);

  // Filter z with a much tighter window
  int z_window_size = 1;  // Original value: 5
  filtered_z = math_utils::movingAverageFilter(z, z_window_size);
  z_rate = math_utils::centralDiff(filtered_z, dt);
  filtered_z_rate = math_utils::movingAverageFilter(z_rate, z_window_size);

  // Wrap yaw again
  wrapped_yaw = math_utils::wrapToPi(unwrapped_yaw);
  filtered_yaw = math_utils::wrapToPi(filtered_yaw);

  // Add full state data into the array
  std::vector<double> x_vec, y_vec, z_vec;
  for (int i = 0; i < num_discrete_pts; i++) {
    State body_state = reduced_plan[i];
    // body_state[2] = filtered_z[i];
    body_state.vel[2] = filtered_z_rate[i];
    FullState body_full_state =
        stateToFullState(body_state, roll, pitch[i], filtered_yaw[i], roll_rate,
                         filtered_pitch_rate[i], filtered_yaw_rate[i]);

    // printFullState(body_full_state);
    full_plan.push_back(body_full_state);
    // x_vec.push_back(body_state.pos[0]);
    // y_vec.push_back(body_state.pos[1]);
    // z_vec.push_back(body_state.pos[2]);
  }
}

FullState stateToFullState(const State &state, double roll, double pitch,
                           double yaw, double roll_rate, double pitch_rate,
                           double yaw_rate) {
  FullState full_state;

  full_state.pos = state.pos;
  full_state.vel = state.vel;
  full_state.ang << roll, pitch, yaw;

  // Convert euler rates to body angular velocity
  Eigen::Vector3d d_rpy, ang_vel;
  d_rpy << roll_rate, pitch_rate, yaw_rate;

  Eigen::Matrix3d d_rpy_to_ang_vel;
  d_rpy_to_ang_vel << cos(pitch) * cos(yaw), -sin(yaw), 0,
      cos(pitch) * sin(yaw), cos(yaw), 0, -sin(pitch), 0, 1;

  ang_vel = d_rpy_to_ang_vel * d_rpy;
  full_state.ang_vel = ang_vel;

  return full_state;
}

void eigenToFullState(const Eigen::VectorXd &s_eig, FullState &s) {
  if (s_eig.size() != 12) {
    ROS_ERROR("Eigen::VectorXd is incorrect size");
  }
  s.pos = s_eig.segment(0, 3);
  s.ang = s_eig.segment(3, 3);
  s.vel = s_eig.segment(6, 3);
  s.ang_vel = s_eig.segment(9, 3);
}

double getPitchFromState(const State &s, const PlannerConfig &planner_config) {
  Eigen::Vector3d surf_norm = getSurfaceNormalFiltered(s, planner_config);

  // Get magnitude of lateral velociy
  double vel = s.vel.head<2>().norm();

  // If velocity is zero or surf norm undefined, assume v is in +x direction
  double v_proj = (vel == 0 || surf_norm[2] <= 0)
                      ? surf_norm[0]
                      : s.vel.head<2>().dot(surf_norm.head<2>()) / vel;

  // set pitch to angle that aligns v_proj with surface normal
  return atan2(v_proj, surf_norm[2]);
}

double getDzFromState(const State &s, const PlannerConfig &planner_config) {
  Eigen::Vector3d surf_norm = getSurfaceNormalFiltered(s, planner_config);

  return ((surf_norm[2] <= 0)
              ? std::numeric_limits<double>::max()
              : -(s.vel.head<2>().dot(surf_norm.head<2>()) / surf_norm[2]));
}

void printState(const State &s) {
  std::cout << "STATE: pos = " << s.pos.transpose()
            << ", vel = " << s.vel.transpose() << std::endl;
}

Eigen::VectorXd fullStateToEigen(const FullState &s) {
  Eigen::VectorXd s_eig(12);
  s_eig.segment(0, 3) = s.pos;
  s_eig.segment(3, 3) = s.ang;
  s_eig.segment(6, 3) = s.vel;
  s_eig.segment(9, 3) = s.ang_vel;
  return s_eig;
}

void vectorToFullState(const std::vector<double> &v, FullState &s) {
  if (v.size() != 12) {
    ROS_ERROR("std::vector<double> is incorrect size");
  }
  s.pos[0] = v[0];
  s.pos[1] = v[1];
  s.pos[2] = v[2];
  s.ang[0] = v[3];
  s.ang[1] = v[4];
  s.ang[2] = v[5];
  s.vel[0] = v[6];
  s.vel[1] = v[7];
  s.vel[2] = v[8];
  s.ang_vel[0] = v[9];
  s.ang_vel[1] = v[10];
  s.ang_vel[2] = v[11];
}

void printGRF(const GRF &grf) {
  std::cout << "GRF: " << grf.transpose() << std::endl;
}

void printFullState(const FullState &s) {
  std::cout << "STATE pos = " << s.pos.transpose()
            << ", vel = " << s.vel.transpose() << std::endl;
  std::cout << "ang = " << s.ang.transpose()
            << ", ang_vel = " << s.ang_vel.transpose() << std::endl;
}

}  // namespace offline_planning_utils
