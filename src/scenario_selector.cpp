// ROS-free ScenarioSelector implementation distilled from Autoware scenario_selector.

#include "scenario_selector.hpp"

#include <algorithm>
#include <cmath>

namespace scenario_selector
{

namespace
{
double distance2d(const Pose & a, const Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  return std::hypot(dx, dy);
}
}  // namespace

ScenarioSelector::ScenarioSelector(const ScenarioSelectorConfig & config) : config_(config) {}

void ScenarioSelector::setLaneDrivingTrajectory(const Trajectory & traj)
{
  lane_driving_trajectory_ = traj;
}

void ScenarioSelector::updateEgoState(const Pose & pose, const Velocity & vel)
{
  ego_pose_ = pose;
  ego_velocity_ = vel;

  // push current speed into buffer for stop judgment
  velocity_buffer_.push_back(vel.linear);
  const std::size_t max_samples =
    static_cast<std::size_t>(std::ceil(config_.stopped_time_threshold / config_.update_period)) +
    1;
  while (velocity_buffer_.size() > max_samples) {
    velocity_buffer_.pop_front();
  }
}

bool ScenarioSelector::isStopped() const
{
  if (velocity_buffer_.empty()) return false;
  return std::all_of(
    velocity_buffer_.begin(), velocity_buffer_.end(), [&](double v) {
      return std::abs(v) < config_.stopped_velocity_threshold;
    });
}

bool ScenarioSelector::isNearTrajectoryEnd() const
{
  if (lane_driving_trajectory_.points.empty()) return false;
  const auto & goal_pose = lane_driving_trajectory_.points.back().pose;
  const double dist = distance2d(goal_pose, ego_pose_);
  return dist < config_.arrived_distance_threshold;
}

ScenarioSelectorOutput ScenarioSelector::update()
{
  // In this simplified port, we always remain in LaneDriving scenario.
  ScenarioSelectorOutput out;
  out.scenario = current_scenario_;
  out.selected_trajectory = lane_driving_trajectory_;
  out.is_stopped = isStopped();
  out.is_near_goal = isNearTrajectoryEnd();
  return out;
}

}  // namespace scenario_selector
