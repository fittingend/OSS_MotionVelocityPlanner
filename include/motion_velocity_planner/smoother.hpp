// Simple velocity profile smoothing helpers.
#pragma once

#include "motion_velocity_planner/types.hpp"

namespace motion_velocity_planner
{

// Apply a simple acceleration limit forward pass to avoid exceeding max_accel/min_accel.
void applyAccelerationLimits(
  Trajectory & traj, const std::vector<double> & arc_lengths, double max_accel, double min_accel);

}  // namespace motion_velocity_planner
