// Longitudinal/trajectory helper utilities (ROS-free).
#pragma once

#include "motion_velocity_planner/types.hpp"

#include <vector>

namespace motion_velocity_planner
{

double calcDistance2d(const Pose & a, const Pose & b);
std::vector<double> calcArcLengths(const Trajectory & traj);

// Find index in trajectory where cumulative arc length exceeds target_s (from start).
std::size_t findIndexAlongArc(const std::vector<double> & arc_lengths, double target_s);

}  // namespace motion_velocity_planner
