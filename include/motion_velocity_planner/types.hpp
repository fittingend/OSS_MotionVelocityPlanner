// Basic plain C++ types for motion velocity planner (ROS-free).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace motion_velocity_planner
{

struct TimeStamp
{
  std::int64_t sec{0};
  std::int64_t nanosec{0};
};

struct Header
{
  TimeStamp stamp{};
  std::string frame_id{"map"};
};

struct Position
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Orientation
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose
{
  Position position;
  Orientation orientation;
};

struct Velocity
{
  double linear{0.0};   // longitudinal velocity in m/s
  double angular{0.0};  // yaw rate in rad/s
};

struct TrajectoryPoint
{
  TimeStamp time_from_start{};
  Pose pose{};
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double acceleration_mps2{0.0};
  double heading_rate_rps{0.0};
  double front_wheel_angle_rad{0.0};
  double rear_wheel_angle_rad{0.0};
};

struct Trajectory
{
  Header header{};
  std::vector<TrajectoryPoint> points;

  bool empty() const { return points.empty(); }
  std::size_t size() const { return points.size(); }
  const TrajectoryPoint & operator[](std::size_t i) const { return points[i]; }
  TrajectoryPoint & operator[](std::size_t i) { return points[i]; }
};

struct EgoState
{
  Pose pose{};
  double velocity_mps{0.0};
  double acceleration_mps2{0.0};
};

struct PlannerParameters
{
  double max_accel{1.5};
  double min_accel{-3.0};
  double max_jerk{2.0};
  double min_speed{0.0};
  double stop_margin{2.0};
};

struct StopCondition
{
  bool has_stop{false};
  double stop_s{0.0};  // arc length along trajectory to stop
};

}  // namespace motion_velocity_planner
