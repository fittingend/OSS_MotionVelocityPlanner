// Simple parsers for reference IO format and writer for trajectory.

#include "io_utils.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace io_utils
{
namespace
{
std::string trim(const std::string & s)
{
  const auto start = s.find_first_not_of(" \t");
  if (start == std::string::npos) return "";
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

double extractNumber(const std::string & line)
{
  const auto colon = line.find(':');
  if (colon == std::string::npos) return 0.0;
  std::string num_str = trim(line.substr(colon + 1));
  std::stringstream ss(num_str);
  double v = 0.0;
  ss >> v;
  return v;
}

long long extractInteger(const std::string & line)
{
  const auto colon = line.find(':');
  if (colon == std::string::npos) return 0;
  std::string num_str = trim(line.substr(colon + 1));
  std::stringstream ss(num_str);
  long long v = 0;
  ss >> v;
  return v;
}
}  // namespace

Trajectory parseTrajectory(const std::filesystem::path & path)
{
  std::ifstream ifs(path);
  std::string line;
  Trajectory traj;
  bool in_points = false;
  bool in_point = false;
  motion_velocity_planner::TrajectoryPoint current{};

  while (std::getline(ifs, line)) {
    const auto tline = trim(line);
    if (!in_points) {
      if (tline.find("sec:") == 0 && traj.header.stamp.sec == 0) {
        traj.header.stamp.sec = extractInteger(line);
        continue;
      }
      if (tline.find("nanosec:") == 0 && traj.header.stamp.nanosec == 0) {
        traj.header.stamp.nanosec = extractInteger(line);
        continue;
      }
      if (tline.find("frame_id:") == 0) {
        const auto colon = line.find(':');
        traj.header.frame_id = trim(line.substr(colon + 1));
        continue;
      }
      if (tline.find("points:") == 0) {
        in_points = true;
        continue;
      }
    } else {
      if (tline.find("- time_from_start") == 0) {
        if (in_point) {
          traj.points.push_back(current);
        }
        in_point = true;
        current = motion_velocity_planner::TrajectoryPoint{};
        continue;
      }
      if (!in_point) continue;

      if (tline.find("sec:") == 0 && current.time_from_start.sec == 0 &&
          current.time_from_start.nanosec == 0) {
        current.time_from_start.sec = extractInteger(line);
        continue;
      }
      if (tline.find("nanosec:") == 0 && current.time_from_start.nanosec == 0) {
        current.time_from_start.nanosec = static_cast<std::int64_t>(extractInteger(line));
        continue;
      }
      if (tline.find("position:") != std::string::npos) {
        std::getline(ifs, line);
        current.pose.position.x = extractNumber(line);
        std::getline(ifs, line);
        current.pose.position.y = extractNumber(line);
        std::getline(ifs, line);
        current.pose.position.z = extractNumber(line);
        continue;
      }
      if (tline.find("orientation:") != std::string::npos) {
        std::getline(ifs, line);
        current.pose.orientation.x = extractNumber(line);
        std::getline(ifs, line);
        current.pose.orientation.y = extractNumber(line);
        std::getline(ifs, line);
        current.pose.orientation.z = extractNumber(line);
        std::getline(ifs, line);
        current.pose.orientation.w = extractNumber(line);
        continue;
      }
      if (tline.find("longitudinal_velocity_mps:") != std::string::npos) {
        current.longitudinal_velocity_mps = extractNumber(line);
        continue;
      }
      if (tline.find("lateral_velocity_mps:") != std::string::npos) {
        current.lateral_velocity_mps = extractNumber(line);
        continue;
      }
      if (tline.find("acceleration_mps2:") != std::string::npos) {
        current.acceleration_mps2 = extractNumber(line);
        continue;
      }
      if (tline.find("heading_rate_rps:") != std::string::npos) {
        current.heading_rate_rps = extractNumber(line);
        continue;
      }
      if (tline.find("front_wheel_angle_rad:") != std::string::npos) {
        current.front_wheel_angle_rad = extractNumber(line);
        continue;
      }
      if (tline.find("rear_wheel_angle_rad:") != std::string::npos) {
        current.rear_wheel_angle_rad = extractNumber(line);
        continue;
      }
    }
  }

  if (in_point) {
    traj.points.push_back(current);
  }
  return traj;
}

EgoState parseLocalization(const std::filesystem::path & path)
{
  std::ifstream ifs(path);
  std::string line;
  EgoState state;

  while (std::getline(ifs, line)) {
    const auto tline = trim(line);
    if (tline.find("sec:") == 0 && state.pose.position.x == 0.0 && state.pose.position.y == 0.0 &&
        state.pose.position.z == 0.0 && state.pose.orientation.w == 0.0) {
      // header stamp not used further; skip
      continue;
    }
    if (tline.find("position:") != std::string::npos) {
      std::getline(ifs, line);
      state.pose.position.x = extractNumber(line);
      std::getline(ifs, line);
      state.pose.position.y = extractNumber(line);
      std::getline(ifs, line);
      state.pose.position.z = extractNumber(line);
      continue;
    }
    if (tline.find("orientation:") != std::string::npos) {
      std::getline(ifs, line);
      state.pose.orientation.x = extractNumber(line);
      std::getline(ifs, line);
      state.pose.orientation.y = extractNumber(line);
      std::getline(ifs, line);
      state.pose.orientation.z = extractNumber(line);
      std::getline(ifs, line);
      state.pose.orientation.w = extractNumber(line);
      continue;
    }
    if (tline.find("twist:") != std::string::npos) {
      // read through to linear x
      while (std::getline(ifs, line)) {
        const auto l = trim(line);
        if (l.find("linear:") != std::string::npos) {
          std::getline(ifs, line);
          state.velocity_mps = extractNumber(line);
          // skip y,z
          std::getline(ifs, line);
          std::getline(ifs, line);
          break;
        }
      }
      continue;
    }
  }
  return state;
}

void writeTrajectory(const std::filesystem::path & path, const Trajectory & traj)
{
  std::ofstream ofs(path);
  ofs << std::fixed << std::setprecision(15);

  ofs << "header:\n";
  ofs << "  stamp:\n";
  ofs << "    sec: " << traj.header.stamp.sec << "\n";
  ofs << "    nanosec: " << traj.header.stamp.nanosec << "\n";
  ofs << "  frame_id: " << traj.header.frame_id << "\n";
  ofs << "points:\n";

  for (const auto & p : traj.points) {
    ofs << "- time_from_start:\n";
    ofs << "    sec: " << p.time_from_start.sec << "\n";
    ofs << "    nanosec: " << p.time_from_start.nanosec << "\n";
    ofs << "  pose:\n";
    ofs << "    position:\n";
    ofs << "      x: " << p.pose.position.x << "\n";
    ofs << "      y: " << p.pose.position.y << "\n";
    ofs << "      z: " << p.pose.position.z << "\n";
    ofs << "    orientation:\n";
    ofs << "      x: " << p.pose.orientation.x << "\n";
    ofs << "      y: " << p.pose.orientation.y << "\n";
    ofs << "      z: " << p.pose.orientation.z << "\n";
    ofs << "      w: " << p.pose.orientation.w << "\n";
    ofs << "  longitudinal_velocity_mps: " << p.longitudinal_velocity_mps << "\n";
    ofs << "  lateral_velocity_mps: " << p.lateral_velocity_mps << "\n";
    ofs << "  acceleration_mps2: " << p.acceleration_mps2 << "\n";
    ofs << "  heading_rate_rps: " << p.heading_rate_rps << "\n";
    ofs << "  front_wheel_angle_rad: " << p.front_wheel_angle_rad << "\n";
    ofs << "  rear_wheel_angle_rad: " << p.rear_wheel_angle_rad << "\n";
  }
}

std::filesystem::path resolveReferencePath(const std::filesystem::path & exe_path)
{
  const auto exe_dir = exe_path.parent_path();
  const auto repo_root = exe_dir.parent_path();
  return repo_root / "reference_IO";
}

}  // namespace io_utils
