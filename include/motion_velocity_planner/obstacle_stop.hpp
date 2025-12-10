// Obstacle/stop line handling to insert stop velocity profile.
#pragma once

#include "motion_velocity_planner/types.hpp"
#include "motion_velocity_planner/longitudinal_utils.hpp"

namespace motion_velocity_planner
{

// Insert a stop at stop_s (arc length). Returns index of stop point in trajectory.
std::size_t insertStopPoint(Trajectory & traj, const std::vector<double> & arc_lengths, double stop_s);

// Apply a simple deceleration profile toward the stop point using min_accel (negative).
void applyStopProfile(
  Trajectory & traj, const std::vector<double> & arc_lengths, std::size_t stop_index,
  double min_accel);

}  // namespace motion_velocity_planner
