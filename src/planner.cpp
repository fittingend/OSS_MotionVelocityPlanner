// Main velocity planning entry point.

#include "motion_velocity_planner/planner.hpp"

#include "motion_velocity_planner/longitudinal_utils.hpp"
#include "motion_velocity_planner/obstacle_stop.hpp"
#include "motion_velocity_planner/smoother.hpp"

#include <algorithm>

namespace motion_velocity_planner
{

Trajectory planVelocityProfile(
  const Trajectory & input_trajectory, const EgoState & /*ego*/,
  const StopCondition & stop_condition, const PlannerParameters & params)
{
  Trajectory traj = input_trajectory;
  if (traj.points.empty()) return traj;

  // Compute arc lengths
  const auto arc_lengths = calcArcLengths(traj);

  // If stop condition exists, insert stop and decel profile
  if (stop_condition.has_stop) {
    const double stop_s = std::max(0.0, stop_condition.stop_s - params.stop_margin);
    const std::size_t stop_idx = insertStopPoint(traj, arc_lengths, stop_s);
    applyStopProfile(traj, arc_lengths, stop_idx, params.min_accel);
  }

  // Apply acceleration limits to smooth profile
  applyAccelerationLimits(traj, arc_lengths, params.max_accel, params.min_accel);

  // Enforce minimum speed non-negativity
  for (auto & p : traj.points) {
    p.longitudinal_velocity_mps = std::max(params.min_speed, p.longitudinal_velocity_mps);
  }

  return traj;
}

}  // namespace motion_velocity_planner
