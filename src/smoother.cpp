// Simple acceleration limiter smoothing.

#include "motion_velocity_planner/smoother.hpp"

#include <algorithm>
#include <cmath>

namespace motion_velocity_planner
{

void applyAccelerationLimits(
  Trajectory & traj, const std::vector<double> & arc_lengths, double max_accel, double min_accel)
{
  if (traj.points.size() < 2) return;

  // Forward pass: limit acceleration increase
  for (std::size_t i = 1; i < traj.points.size(); ++i) {
    const double ds = arc_lengths[i] - arc_lengths[i - 1];
    if (ds <= 1e-4) continue;
    const double v_prev = traj.points[i - 1].longitudinal_velocity_mps;
    const double v_curr = traj.points[i].longitudinal_velocity_mps;
    const double max_v = std::sqrt(std::max(0.0, v_prev * v_prev + 2.0 * max_accel * ds));
    const double min_v = std::sqrt(std::max(0.0, v_prev * v_prev + 2.0 * min_accel * ds));
    const double clamped_v = std::clamp(v_curr, min_v, max_v);
    traj.points[i].longitudinal_velocity_mps = clamped_v;
  }

  // Backward pass: limit braking
  for (std::size_t i = traj.points.size() - 1; i > 0; --i) {
    const double ds = arc_lengths[i] - arc_lengths[i - 1];
    if (ds <= 1e-4) continue;
    const double v_next = traj.points[i].longitudinal_velocity_mps;
    const double v_curr = traj.points[i - 1].longitudinal_velocity_mps;
    const double max_v = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * max_accel * ds));
    const double min_v = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * min_accel * ds));
    const double clamped_v = std::clamp(v_curr, min_v, max_v);
    traj.points[i - 1].longitudinal_velocity_mps = clamped_v;
  }
}

}  // namespace motion_velocity_planner
