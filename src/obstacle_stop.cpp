// Stop point insertion and deceleration profile application.

#include "motion_velocity_planner/obstacle_stop.hpp"

#include <algorithm>
#include <cmath>

namespace motion_velocity_planner
{

std::size_t insertStopPoint(
  Trajectory & traj, const std::vector<double> & arc_lengths, double stop_s)
{
  if (traj.points.empty()) return 0;
  const std::size_t idx = findIndexAlongArc(arc_lengths, stop_s);
  traj.points[idx].longitudinal_velocity_mps = 0.0;
  for (std::size_t i = idx + 1; i < traj.points.size(); ++i) {
    traj.points[i].longitudinal_velocity_mps = 0.0;
  }
  return idx;
}

void applyStopProfile(
  Trajectory & traj, const std::vector<double> & arc_lengths, std::size_t stop_index,
  double min_accel)
{
  if (traj.points.empty() || stop_index >= traj.points.size()) return;

  // Backward from stop to start: limit speed based on braking distance v^2 = 2 a s
  traj.points[stop_index].longitudinal_velocity_mps = 0.0;
  for (std::size_t i = stop_index; i > 0; --i) {
    const double ds = arc_lengths[i] - arc_lengths[i - 1];
    if (ds <= 1e-4) continue;
    const double v_next = traj.points[i].longitudinal_velocity_mps;
    const double max_v = std::sqrt(std::max(0.0, v_next * v_next - 2.0 * min_accel * ds));
    traj.points[i - 1].longitudinal_velocity_mps =
      std::min(traj.points[i - 1].longitudinal_velocity_mps, max_v);
  }
}

}  // namespace motion_velocity_planner
