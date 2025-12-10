// Utilities to parse reference text inputs and write trajectory outputs.
#pragma once

#include "motion_velocity_planner/types.hpp"

#include <filesystem>
#include <vector>

namespace io_utils
{
using motion_velocity_planner::EgoState;
using motion_velocity_planner::Trajectory;

Trajectory parseTrajectory(const std::filesystem::path & path);
EgoState parseLocalization(const std::filesystem::path & path);
void writeTrajectory(const std::filesystem::path & path, const Trajectory & traj);

std::filesystem::path resolveReferencePath(const std::filesystem::path & exe_path);

}  // namespace io_utils
