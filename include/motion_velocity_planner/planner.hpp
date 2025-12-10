// Main interface for the ROS-free motion velocity planner.
#pragma once

#include "motion_velocity_planner/types.hpp"

namespace motion_velocity_planner
{

Trajectory planVelocityProfile(
  const Trajectory & input_trajectory, const EgoState & ego, const StopCondition & stop_condition,
  const PlannerParameters & params);

}  // namespace motion_velocity_planner
