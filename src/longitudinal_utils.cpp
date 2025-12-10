// Longitudinal helper utilities.

#include "motion_velocity_planner/longitudinal_utils.hpp"

#include <cmath>

namespace motion_velocity_planner
{

double calcDistance2d(const Pose & a, const Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  return std::hypot(dx, dy);
}

std::vector<double> calcArcLengths(const Trajectory & traj)
{
  std::vector<double> arc;
  arc.reserve(traj.points.size());
  double sum = 0.0;
  for (std::size_t i = 0; i < traj.points.size(); ++i) {
    if (i == 0) {
      arc.push_back(0.0);
    } else {
      sum += calcDistance2d(traj.points[i - 1].pose, traj.points[i].pose);
      arc.push_back(sum);
    }
  }
  return arc;
}

std::size_t findIndexAlongArc(const std::vector<double> & arc_lengths, double target_s)
{
  for (std::size_t i = 0; i < arc_lengths.size(); ++i) {
    if (arc_lengths[i] >= target_s) return i;
  }
  return arc_lengths.size() - 1;
}

}  // namespace motion_velocity_planner
