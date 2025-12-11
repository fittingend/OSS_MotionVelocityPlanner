#include "io_utils.hpp"
#include "jsonl_utils.hpp"
#include "motion_velocity_planner/adapter.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{

struct CliOptions
{
  std::filesystem::path output_dir{"output"};
  std::filesystem::path scenario_dir;
  int scenario_id{0};
  bool reference_mode{false};
  bool help{false};
};

std::string trim(const std::string & line)
{
  const auto start = line.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = line.find_last_not_of(" \t\r\n");
  return line.substr(start, end - start + 1);
}

long long parseIntegerAfterColon(const std::string & line)
{
  const auto colon = line.find(':');
  if (colon == std::string::npos) {
    return 0;
  }
  const auto value = trim(line.substr(colon + 1));
  try {
    return std::stoll(value);
  } catch (...) {
    return 0;
  }
}

motion_velocity_planner::Header parseReferenceLocalizationHeader(const std::filesystem::path & path)
{
  motion_velocity_planner::Header header;
  std::ifstream ifs(path);
  if (!ifs) {
    return header;
  }
  std::string line;
  bool in_header = false;
  bool in_stamp = false;
  while (std::getline(ifs, line)) {
    const auto entry = trim(line);
    if (!in_header) {
      if (entry == "header:") {
        in_header = true;
      }
      continue;
    }
    if (!in_stamp) {
      if (entry == "stamp:") {
        in_stamp = true;
        continue;
      }
      if (entry.rfind("frame_id:", 0) == 0) {
        header.frame_id = trim(line.substr(line.find(':') + 1));
        break;
      }
      continue;
    }
    if (entry.rfind("sec:", 0) == 0) {
      header.stamp.sec = parseIntegerAfterColon(line);
      continue;
    }
    if (entry.rfind("nanosec:", 0) == 0) {
      header.stamp.nanosec = parseIntegerAfterColon(line);
      continue;
    }
    if (entry.rfind("frame_id:", 0) == 0) {
      header.frame_id = trim(line.substr(line.find(':') + 1));
      break;
    }
  }
  return header;
}

motion_velocity_planner::LocalizationKinematicState loadReferenceLocalizationState(
  const std::filesystem::path & path)
{
  const motion_velocity_planner::EgoState ego = io_utils::parseLocalization(path);
  motion_velocity_planner::LocalizationKinematicState state;
  state.pose = ego.pose;
  state.twist.linear = ego.velocity_mps;
  state.acceleration_mps2 = ego.acceleration_mps2;
  state.header = parseReferenceLocalizationHeader(path);
  if (state.header.frame_id.empty()) {
    state.header.frame_id = "map";
  }
  return state;
}

void printUsage(const char * program)
{
  std::cout << "Usage: " << program << " [--reference] [--scenario-id N] [--output-dir DIR]\n"
            << "Options:\n"
            << "  --reference               Run the original reference TXT mode\n"
            << "  --scenario-id N           Select scenario ID (1,2,3) to choose Autoware JSONL inputs\n"
            << "  --output-dir DIR          Location where JSONL output will be stored (default: output)\n"
            << "  --help, -h                Show this help message\n";
}

std::optional<int> parseInteger(const std::string & text)
{
  try {
    return std::stoi(text);
  } catch (...) {
    return std::nullopt;
  }
}

const std::filesystem::path kScenario1Dir =
  "/home/sujin/Desktop/OSS_TEST/Scenario1_Straight_json";
const std::filesystem::path kScenario2Dir =
  "/home/sujin/Desktop/OSS_TEST/Scenario2_Corner_json";
const std::filesystem::path kScenario3Dir =
  "/home/sujin/Desktop/OSS_TEST/Scenario3_Straight_object_json";
constexpr int kScenarioIdDefault{1};

std::filesystem::path resolveScenarioDir(int scenario_id)
{
  switch (scenario_id) {
    case 2:
      return kScenario2Dir;
    case 3:
      return kScenario3Dir;
    default:
      return kScenario1Dir;
  }
}

std::optional<CliOptions> parseArguments(int argc, char ** argv)
{
  CliOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      options.help = true;
      return options;
    }
    if (arg == "--reference") {
      options.reference_mode = true;
      continue;
    }
    if (arg == "--scenario-id") {
      if (++i >= argc) {
        std::cerr << "--scenario-id requires a numeric identifier\n";
        return std::nullopt;
      }
      if (const auto id = parseInteger(argv[i])) {
        options.scenario_id = *id;
        options.scenario_dir = resolveScenarioDir(*id);
        continue;
      }
      std::cerr << "Invalid scenario id: " << argv[i] << "\n";
      return std::nullopt;
    }
    if (arg == "--output-dir") {
      if (++i >= argc) {
        std::cerr << "--output-dir requires a directory\n";
        return std::nullopt;
      }
      options.output_dir = argv[i];
      continue;
    }
    std::cerr << "Unknown option: " << arg << "\n";
    return std::nullopt;
  }
  return options;
}

std::string makeOutputFileName(int scenario_id)
{
  std::ostringstream oss;
  oss << "planning__scenario_planning__lane_driving__motion_planning__motion_velocity_planner__trajectory__scenario"
      << scenario_id << ".jsonl";
  return oss.str();
}

int runReferenceMode(const std::filesystem::path & executable)
{
  const std::filesystem::path reference_dir = io_utils::resolveReferencePath(executable);
  const auto localization_path = reference_dir / "localization_kinematicstate.txt";
  const auto path_optimizer_path = reference_dir / "path_optimizer_NonObj.txt";
  const auto output_path = reference_dir / "motion_velocity_planner.txt";

  const auto localization_state = loadReferenceLocalizationState(localization_path);
  motion_velocity_planner::PathOptimizerResult path_data;
  path_data.trajectory = io_utils::parseTrajectory(path_optimizer_path);

  const auto planned = motion_velocity_planner::compute_motion_velocity(localization_state, {}, path_data);
  io_utils::writeTrajectory(output_path, planned.trajectory);
  std::cout << "Wrote " << output_path << "\n";
  return 0;
}

int runJsonMode(const CliOptions & options, const std::filesystem::path & scenario_json_dir, int scenario_id)
{
  const auto localization_file = scenario_json_dir / "localization__kinematic_state.jsonl";
  const auto objects_file = scenario_json_dir / "perception__object_recognition__objects.jsonl";
  const auto path_file =
    scenario_json_dir /
    "planning__scenario_planning__lane_driving__motion_planning__path_optimizer__trajectory.jsonl";

  if (!std::filesystem::exists(scenario_json_dir)) {
    std::cerr << "Scenario directory not found: " << scenario_json_dir << "\n";
    return 1;
  }

  std::vector<jsonl_utils::TimeStampedFrame<motion_velocity_planner::LocalizationKinematicState>>
    localization_frames;
  std::vector<jsonl_utils::TimeStampedFrame<std::vector<motion_velocity_planner::Object>>> object_frames;
  std::vector<jsonl_utils::TimeStampedFrame<motion_velocity_planner::PathOptimizerResult>> path_frames;

  try {
    localization_frames = jsonl_utils::loadLocalizationFrames(localization_file);
    object_frames = jsonl_utils::loadObjectFrames(objects_file);
    path_frames = jsonl_utils::loadPathOptimizerFrames(path_file);
  } catch (const std::exception & ex) {
    std::cerr << "Failed to load JSONL data: " << ex.what() << "\n";
    return 1;
  }

  if (localization_frames.empty()) {
    std::cerr << "No localization frames found in " << localization_file << "\n";
    return 1;
  }
  if (path_frames.empty()) {
    std::cerr << "No path optimizer frames found in " << path_file << "\n";
    return 1;
  }

  std::filesystem::create_directories(options.output_dir);
  const auto output_path = options.output_dir / makeOutputFileName(scenario_id);
  std::ofstream ofs(output_path);
  if (!ofs) {
    std::cerr << "Unable to write to " << output_path << "\n";
    return 1;
  }

  static const std::vector<motion_velocity_planner::Object> kEmptyObjects{};
  std::size_t written = 0;

  for (const auto & frame : path_frames) {
    const auto localization_index =
      jsonl_utils::findClosestFrameIndex(localization_frames, frame.stamp);
    if (!localization_index) {
      continue;
    }

    const auto object_index = jsonl_utils::findClosestFrameIndex(object_frames, frame.stamp);
    const std::vector<motion_velocity_planner::Object> & objects =
      object_index ? object_frames[*object_index].data : kEmptyObjects;

    const auto & localization_state = localization_frames[*localization_index].data;
    const auto & path_result = frame.data;
    const auto planned =
      motion_velocity_planner::compute_motion_velocity(localization_state, objects, path_result);
    const auto line = jsonl_utils::serializeMotionVelocityPlannerLine(
      planned, path_result.trajectory.header, written, scenario_id);
    ofs << line << "\n";
    ++written;
  }

  std::cout << "Wrote " << output_path << " (" << written << " frames)\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::filesystem::path exe_path = (argc > 0) ? std::filesystem::path(argv[0]) : "";
  auto options = parseArguments(argc, argv);
  if (!options) {
    return 1;
  }
  if (options->help) {
    printUsage(argv[0]);
    return 0;
  }
  if (options->reference_mode) {
    return runReferenceMode(exe_path);
  }
  const int active_scenario_id =
    options->scenario_id > 0 ? options->scenario_id : kScenarioIdDefault;
  if (options->scenario_dir.empty()) {
    options->scenario_dir = resolveScenarioDir(active_scenario_id);
  }
  return runJsonMode(*options, options->scenario_dir, active_scenario_id);
}
