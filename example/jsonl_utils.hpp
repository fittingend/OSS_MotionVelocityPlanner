#pragma once

#include "motion_velocity_planner/adapter.hpp"

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

namespace jsonl_utils
{

template <typename T>
struct TimeStampedFrame
{
  motion_velocity_planner::TimeStamp stamp{};
  T data{};
};

std::vector<TimeStampedFrame<motion_velocity_planner::LocalizationKinematicState>>
loadLocalizationFrames(const std::filesystem::path & jsonl_path);

std::vector<TimeStampedFrame<std::vector<motion_velocity_planner::Object>>>
loadObjectFrames(const std::filesystem::path & jsonl_path);

std::vector<TimeStampedFrame<motion_velocity_planner::PathOptimizerResult>>
loadPathOptimizerFrames(const std::filesystem::path & jsonl_path);

template <typename T>
std::optional<std::size_t> findClosestFrameIndex(
  const std::vector<TimeStampedFrame<T>> & frames,
  const motion_velocity_planner::TimeStamp & stamp)
{
  if (frames.empty()) {
    return std::nullopt;
  }
  const auto target_ns =
    stamp.sec * static_cast<std::int64_t>(1000000000LL) + stamp.nanosec;
  const auto timestamp = [](const TimeStampedFrame<T> & frame) {
    return frame.stamp.sec * static_cast<std::int64_t>(1000000000LL) + frame.stamp.nanosec;
  };
  auto it = std::lower_bound(
    frames.begin(), frames.end(), target_ns,
    [&](const TimeStampedFrame<T> & frame, std::int64_t value) {
      return timestamp(frame) < value;
    });
  if (it == frames.end()) {
    return frames.size() - 1;
  }
  if (it == frames.begin()) {
    return 0;
  }
  const auto prev = std::prev(it);
  const auto next_diff = std::llabs(timestamp(*it) - target_ns);
  const auto prev_diff = std::llabs(timestamp(*prev) - target_ns);
  return (prev_diff <= next_diff) ? std::distance(frames.begin(), prev)
                                  : std::distance(frames.begin(), it);
}

std::string serializeMotionVelocityPlannerLine(
  const motion_velocity_planner::MotionVelocityPlannerOutput & output,
  const motion_velocity_planner::Header & header,
  std::size_t frame_index,
  int scenario_id);

}  // namespace jsonl_utils
