#include "motion_velocity_planner/adapter.hpp"

#include "motion_velocity_planner/longitudinal_utils.hpp"
#include "motion_velocity_planner/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace motion_velocity_planner
{

namespace
{
EgoState makeEgoState(const LocalizationKinematicState & localization)
{
  EgoState ego{};
  ego.pose = localization.pose;
  ego.velocity_mps = localization.twist.linear;
  ego.acceleration_mps2 = localization.acceleration_mps2;
  return ego;
}

StopCondition buildStopCondition(
  const Trajectory & trajectory, const std::vector<Object> & objects)
{
  StopCondition stop{};
  if (trajectory.points.empty() || objects.empty()) {
    return stop;
  }

  const auto arc_lengths = calcArcLengths(trajectory);
  double best_distance = std::numeric_limits<double>::infinity();
  double best_s = 0.0;

  for (const auto & object : objects) {
    if (object.existence_probability <= 0.0) {
      continue;
    }
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
      const double dist = calcDistance2d(trajectory.points[i].pose, object.pose);
      if (dist < best_distance) {
        best_distance = dist;
        best_s = arc_lengths[i];
      }
    }
  }

  constexpr double kStopDistanceThreshold = 4.0;
  constexpr double kStopPadding = 0.5;

  if (best_distance < kStopDistanceThreshold) {
    stop.has_stop = true;
    stop.stop_s = std::max(0.0, best_s - kStopPadding);
  }
  return stop;
}

}  // namespace

MotionVelocityPlannerOutput compute_motion_velocity(
  const LocalizationKinematicState & localization,
  const std::vector<Object> & objects,
  const PathOptimizerResult & path_optimizer)
{
  MotionVelocityPlannerOutput output{};
  if (path_optimizer.trajectory.points.empty()) {
    return output;
  }

  const EgoState ego = makeEgoState(localization);

  PlannerParameters params;
  params.max_accel = 1.5;
  params.min_accel = -3.0;
  params.min_speed = 0.0;
  params.stop_margin = 0.0;

  const StopCondition stop_condition =
    buildStopCondition(path_optimizer.trajectory, objects);

  output.trajectory = planVelocityProfile(
    path_optimizer.trajectory, ego, stop_condition, params);
  return output;
}

}  // namespace motion_velocity_planner
