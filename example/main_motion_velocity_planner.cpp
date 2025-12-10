// Parse reference inputs, run motion velocity planner, and write output matching reference format.

#include "io_utils.hpp"
#include "motion_velocity_planner/planner.hpp"

#include <filesystem>
#include <iostream>

using motion_velocity_planner::EgoState;
using motion_velocity_planner::PlannerParameters;
using motion_velocity_planner::StopCondition;
using motion_velocity_planner::Trajectory;

int main(int argc, char ** argv)
{
  const std::filesystem::path exe_path = (argc > 0) ? std::filesystem::path(argv[0]) : "";
  const auto ref_dir = io_utils::resolveReferencePath(exe_path);

  const auto localization_path = ref_dir / "localization_kinematicstate.txt";
  const auto path_optimizer_path = ref_dir / "path_optimizer_NonObj.txt";
  const auto output_path = ref_dir / "motion_velocity_planner.txt";

  const EgoState ego = io_utils::parseLocalization(localization_path);
  const Trajectory path_traj = io_utils::parseTrajectory(path_optimizer_path);

  PlannerParameters params;
  params.max_accel = 1.5;
  params.min_accel = -3.0;
  params.min_speed = 0.0;
  params.stop_margin = 0.0;  // no extra margin for non-obstacle case

  StopCondition stop_cond;
  stop_cond.has_stop = false;  // scenario 1: no obstacle

  const auto planned = motion_velocity_planner::planVelocityProfile(
    path_traj, ego, stop_cond, params);

  io_utils::writeTrajectory(output_path, planned);
  std::cout << "Wrote " << output_path << "\n";
  return 0;
}
