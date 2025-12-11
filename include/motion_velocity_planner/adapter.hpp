#pragma once

#include "motion_velocity_planner/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace motion_velocity_planner
{

struct LocalizationKinematicState
{
  Header header{};
  Pose pose{};
  Velocity twist{};
  double acceleration_mps2{0.0};
};

struct Object
{
  std::string uuid;
  double existence_probability{0.0};
  Pose pose{};
  Velocity velocity{};
  double acceleration_mps2{0.0};
  double length{0.0};
  double width{0.0};
  double height{0.0};
  double confidence{0.0};
};

struct PathOptimizerResult
{
  Trajectory trajectory{};
};

struct MotionVelocityPlannerOutput
{
  Trajectory trajectory{};
};

MotionVelocityPlannerOutput compute_motion_velocity(
  const LocalizationKinematicState & localization,
  const std::vector<Object> & objects,
  const PathOptimizerResult & path_optimizer);

}  // namespace motion_velocity_planner
