#include "jsonl_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace
{

enum class JsonType
{
  Null,
  Bool,
  Number,
  String,
  Object,
  Array
};

struct JsonValue
{
  JsonType type{JsonType::Null};
  bool bool_value{false};
  double number_value{0.0};
  std::string string_value;
  std::vector<JsonValue> array;
  std::vector<std::pair<std::string, JsonValue>> object;

  const JsonValue * getMember(const std::string_view key) const
  {
    if (type != JsonType::Object) {
      return nullptr;
    }
    for (const auto & member : object) {
      if (member.first == key) {
        return &member.second;
      }
    }
    return nullptr;
  }
};

class JsonParser
{
public:
  explicit JsonParser(const std::string & text) : text_(text), idx_(0) {}

  JsonValue parse()
  {
    const JsonValue value = parseValue();
    skipWhitespace();
    if (idx_ != text_.size()) {
      throw std::runtime_error("Unexpected characters after JSON value");
    }
    return value;
  }

private:
  const std::string & text_;
  std::size_t idx_;

  void skipWhitespace()
  {
    while (idx_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[idx_]))) {
      ++idx_;
    }
  }

  char peek() const { return (idx_ < text_.size()) ? text_[idx_] : '\0'; }

  char get()
  {
    if (idx_ >= text_.size()) {
      throw std::runtime_error("Unexpected end of JSON");
    }
    return text_[idx_++];
  }

  JsonValue parseValue()
  {
    skipWhitespace();
    const char c = peek();
    if (c == '{') {
      return parseObject();
    }
    if (c == '[') {
      return parseArray();
    }
    if (c == '"') {
      return parseString();
    }
    if (c == 't') {
      return parseTrue();
    }
    if (c == 'f') {
      return parseFalse();
    }
    if (c == 'n') {
      return parseNull();
    }
    return parseNumber();
  }

  JsonValue parseObject()
  {
    JsonValue value;
    value.type = JsonType::Object;
    get();  // consume '{'
    skipWhitespace();
    if (peek() == '}') {
      get();
      return value;
    }
    while (true) {
      skipWhitespace();
      const JsonValue key = parseString();
      skipWhitespace();
      if (get() != ':') {
        throw std::runtime_error("Expected ':' in object");
      }
      JsonValue element = parseValue();
      value.object.emplace_back(key.string_value, std::move(element));
      skipWhitespace();
      const char delim = peek();
      if (delim == ',') {
        get();
        continue;
      }
      if (delim == '}') {
        get();
        break;
      }
      throw std::runtime_error("Expected ',' or '}' in object");
    }
    return value;
  }

  JsonValue parseArray()
  {
    JsonValue value;
    value.type = JsonType::Array;
    get();  // consume '['
    skipWhitespace();
    if (peek() == ']') {
      get();
      return value;
    }
    while (true) {
      JsonValue element = parseValue();
      value.array.push_back(std::move(element));
      skipWhitespace();
      const char delim = peek();
      if (delim == ',') {
        get();
        continue;
      }
      if (delim == ']') {
        get();
        break;
      }
      throw std::runtime_error("Expected ',' or ']' in array");
    }
    return value;
  }

  JsonValue parseString()
  {
    JsonValue value;
    value.type = JsonType::String;
    get();  // consume '"'
    std::string result;
    while (true) {
      if (idx_ >= text_.size()) {
        throw std::runtime_error("Unterminated JSON string");
      }
      char c = get();
      if (c == '"') {
        break;
      }
      if (c == '\\') {
        if (idx_ >= text_.size()) {
          throw std::runtime_error("Invalid escape in JSON string");
        }
        char esc = get();
        switch (esc) {
          case '"':
            result += '"';
            break;
          case '\\':
            result += '\\';
            break;
          case '/':
            result += '/';
            break;
          case 'b':
            result += '\b';
            break;
          case 'f':
            result += '\f';
            break;
          case 'n':
            result += '\n';
            break;
          case 'r':
            result += '\r';
            break;
          case 't':
            result += '\t';
            break;
          case 'u': {
            int code = 0;
            for (int i = 0; i < 4; ++i) {
              const char hex = get();
              code <<= 4;
              if (hex >= '0' && hex <= '9') {
                code += hex - '0';
              } else if (hex >= 'a' && hex <= 'f') {
                code += 10 + (hex - 'a');
              } else if (hex >= 'A' && hex <= 'F') {
                code += 10 + (hex - 'A');
              } else {
                throw std::runtime_error("Invalid Unicode escape");
              }
            }
            if (code < 0x80) {
              result += static_cast<char>(code);
            }
            break;
          }
          default:
            break;
        }
        continue;
      }
      result += c;
    }
    value.string_value = std::move(result);
    return value;
  }

  JsonValue parseNumber()
  {
    JsonValue value;
    value.type = JsonType::Number;
    const std::size_t start = idx_;
    if (peek() == '-') {
      ++idx_;
    }
    if (peek() == '0') {
      ++idx_;
    } else if (std::isdigit(static_cast<unsigned char>(peek()))) {
      while (std::isdigit(static_cast<unsigned char>(peek()))) {
        ++idx_;
      }
    } else {
      throw std::runtime_error("Invalid number");
    }
    if (peek() == '.') {
      ++idx_;
      if (!std::isdigit(static_cast<unsigned char>(peek()))) {
        throw std::runtime_error("Invalid number");
      }
      while (std::isdigit(static_cast<unsigned char>(peek()))) {
        ++idx_;
      }
    }
    if (peek() == 'e' || peek() == 'E') {
      ++idx_;
      if (peek() == '+' || peek() == '-') {
        ++idx_;
      }
      if (!std::isdigit(static_cast<unsigned char>(peek()))) {
        throw std::runtime_error("Invalid number");
      }
      while (std::isdigit(static_cast<unsigned char>(peek()))) {
        ++idx_;
      }
    }
    const auto token = text_.substr(start, idx_ - start);
    value.number_value = std::stod(token);
    return value;
  }

  JsonValue parseTrue()
  {
    JsonValue value;
    value.type = JsonType::Bool;
    if (text_.compare(idx_, 4, "true") != 0) {
      throw std::runtime_error("Invalid literal");
    }
    idx_ += 4;
    value.bool_value = true;
    return value;
  }

  JsonValue parseFalse()
  {
    JsonValue value;
    value.type = JsonType::Bool;
    if (text_.compare(idx_, 5, "false") != 0) {
      throw std::runtime_error("Invalid literal");
    }
    idx_ += 5;
    value.bool_value = false;
    return value;
  }

  JsonValue parseNull()
  {
    JsonValue value;
    value.type = JsonType::Null;
    if (text_.compare(idx_, 4, "null") != 0) {
      throw std::runtime_error("Invalid literal");
    }
    idx_ += 4;
    return value;
  }
};

const JsonValue * findChild(
  const JsonValue & node, std::initializer_list<std::string_view> path)
{
  const JsonValue * current = &node;
  for (const auto & segment : path) {
    if (!current || current->type != JsonType::Object) {
      return nullptr;
    }
    current = current->getMember(segment);
  }
  return current;
}

std::optional<double> asDouble(const JsonValue * value)
{
  if (!value) {
    return std::nullopt;
  }
  if (value->type == JsonType::Number) {
    return value->number_value;
  }
  return std::nullopt;
}

std::optional<std::int64_t> asInteger(const JsonValue * value)
{
  if (const auto number = asDouble(value)) {
    return static_cast<std::int64_t>(*number);
  }
  return std::nullopt;
}

std::string toString(const JsonValue * value)
{
  if (!value) {
    return {};
  }
  if (value->type == JsonType::String) {
    return value->string_value;
  }
  return {};
}

motion_velocity_planner::Header parseHeader(const JsonValue * node)
{
  motion_velocity_planner::Header header;
  if (!node) {
    return header;
  }
  if (const auto * stamp = findChild(*node, {"stamp", "sec"})) {
    if (const auto sec = asInteger(stamp)) {
      header.stamp.sec = *sec;
    }
  }
  if (const auto * stamp = findChild(*node, {"stamp", "nanosec"})) {
    if (const auto nano = asInteger(stamp)) {
      header.stamp.nanosec = *nano;
    }
  }
  if (const auto * frame_id = node->getMember("frame_id")) {
    header.frame_id = toString(frame_id);
  }
  return header;
}

motion_velocity_planner::Pose parsePose(const JsonValue & node)
{
  motion_velocity_planner::Pose pose;
  if (const auto * position = findChild(node, {"position"})) {
    if (const auto * x = position->getMember("x")) {
      if (const auto val = asDouble(x)) {
        pose.position.x = *val;
      }
    }
    if (const auto * y = position->getMember("y")) {
      if (const auto val = asDouble(y)) {
        pose.position.y = *val;
      }
    }
    if (const auto * z = position->getMember("z")) {
      if (const auto val = asDouble(z)) {
        pose.position.z = *val;
      }
    }
  }
  if (const auto * orientation = findChild(node, {"orientation"})) {
    if (const auto * x = orientation->getMember("x")) {
      if (const auto val = asDouble(x)) {
        pose.orientation.x = *val;
      }
    }
    if (const auto * y = orientation->getMember("y")) {
      if (const auto val = asDouble(y)) {
        pose.orientation.y = *val;
      }
    }
    if (const auto * z = orientation->getMember("z")) {
      if (const auto val = asDouble(z)) {
        pose.orientation.z = *val;
      }
    }
    if (const auto * w = orientation->getMember("w")) {
      if (const auto val = asDouble(w)) {
        pose.orientation.w = *val;
      }
    }
  }
  return pose;
}

motion_velocity_planner::TrajectoryPoint parseTrajectoryPoint(const JsonValue & node)
{
  motion_velocity_planner::TrajectoryPoint point;
  if (const auto * time_node = node.getMember("time_from_start")) {
    if (const auto * sec = time_node->getMember("sec")) {
      if (const auto value = asInteger(sec)) {
        point.time_from_start.sec = *value;
      }
    }
    if (const auto * nanosec = time_node->getMember("nanosec")) {
      if (const auto value = asInteger(nanosec)) {
        point.time_from_start.nanosec = *value;
      }
    }
  }
  if (const auto * pose_node = node.getMember("pose")) {
    point.pose = parsePose(*pose_node);
  }
  if (const auto * long_vel = node.getMember("longitudinal_velocity_mps")) {
    if (const auto value = asDouble(long_vel)) {
      point.longitudinal_velocity_mps = *value;
    }
  }
  if (const auto * lat_vel = node.getMember("lateral_velocity_mps")) {
    if (const auto value = asDouble(lat_vel)) {
      point.lateral_velocity_mps = *value;
    }
  }
  if (const auto * accel = node.getMember("acceleration_mps2")) {
    if (const auto value = asDouble(accel)) {
      point.acceleration_mps2 = *value;
    }
  }
  if (const auto * heading_rate = node.getMember("heading_rate_rps")) {
    if (const auto value = asDouble(heading_rate)) {
      point.heading_rate_rps = *value;
    }
  }
  if (const auto * front_angle = node.getMember("front_wheel_angle_rad")) {
    if (const auto value = asDouble(front_angle)) {
      point.front_wheel_angle_rad = *value;
    }
  }
  if (const auto * rear_angle = node.getMember("rear_wheel_angle_rad")) {
    if (const auto value = asDouble(rear_angle)) {
      point.rear_wheel_angle_rad = *value;
    }
  }
  return point;
}

motion_velocity_planner::LocalizationKinematicState parseLocalizationStateJson(const JsonValue & root)
{
  motion_velocity_planner::LocalizationKinematicState state;
  if (const auto * header = root.getMember("header")) {
    state.header = parseHeader(header);
  }
  if (const auto * pose_node = findChild(root, {"pose", "pose"})) {
    state.pose = parsePose(*pose_node);
  } else if (const auto * pose_node = root.getMember("pose")) {
    state.pose = parsePose(*pose_node);
  }
  if (const auto * linear = findChild(root, {"twist", "twist", "linear", "x"})) {
    if (const auto value = asDouble(linear)) {
      state.twist.linear = *value;
    }
  }
  if (const auto * angular = findChild(root, {"twist", "twist", "angular", "z"})) {
    if (const auto value = asDouble(angular)) {
      state.twist.angular = *value;
    }
  }
  if (const auto * accel = findChild(root, {"accel", "accel", "linear", "x"})) {
    if (const auto value = asDouble(accel)) {
      state.acceleration_mps2 = *value;
    }
  }
  return state;
}

motion_velocity_planner::Object parseObjectJson(const JsonValue & node)
{
  motion_velocity_planner::Object object;
  if (const auto * existence = node.getMember("existence_probability")) {
    if (const auto value = asDouble(existence)) {
      object.existence_probability = *value;
    }
  }
  if (const auto * uuid_array =
        findChild(node, {"object_id", "uuid"})) {
    if (uuid_array->type == JsonType::Array) {
      std::ostringstream uuid;
      bool first = true;
      for (const auto & id_value : uuid_array->array) {
        if (const auto value = asInteger(&id_value)) {
          if (!first) {
            uuid << '-';
          }
          uuid << *value;
          first = false;
        }
      }
      object.uuid = uuid.str();
    }
  }
  if (const auto * pose_node =
        findChild(node, {"kinematics", "initial_pose_with_covariance", "pose"})) {
    object.pose = parsePose(*pose_node);
  }
  if (const auto * velocity_node =
        findChild(node, {"kinematics", "initial_twist_with_covariance", "twist", "linear"})) {
    if (const auto * x = velocity_node->getMember("x")) {
      if (const auto value = asDouble(x)) {
        object.velocity.linear = *value;
      }
    }
    if (const auto * z = velocity_node->getMember("z")) {
      if (const auto value = asDouble(z)) {
        object.velocity.angular = *value;
      }
    }
  }
  if (const auto * accel_node =
        findChild(node, {"kinematics", "initial_acceleration_with_covariance", "accel", "linear", "x"})) {
    if (const auto value = asDouble(accel_node)) {
      object.acceleration_mps2 = *value;
    }
  }
  if (const auto * shape_node = node.getMember("shape")) {
    if (const auto * dimensions = shape_node->getMember("dimensions")) {
      if (const auto * length = dimensions->getMember("x")) {
        if (const auto value = asDouble(length)) {
          object.length = *value;
        }
      }
      if (const auto * width = dimensions->getMember("y")) {
        if (const auto value = asDouble(width)) {
          object.width = *value;
        }
      }
      if (const auto * height = dimensions->getMember("z")) {
        if (const auto value = asDouble(height)) {
          object.height = *value;
        }
      }
    }
  }
  if (const auto * predicted_paths = findChild(node, {"kinematics", "predicted_paths"})) {
    if (predicted_paths->type == JsonType::Array && !predicted_paths->array.empty()) {
      const auto & first_path = predicted_paths->array.front();
      if (const auto * confidence = first_path.getMember("confidence")) {
        if (const auto value = asDouble(confidence)) {
          object.confidence = *value;
        }
      }
    }
  }
  return object;
}

motion_velocity_planner::PathOptimizerResult parsePathOptimizerJson(const JsonValue & root)
{
  motion_velocity_planner::PathOptimizerResult result;
  const JsonValue * trajectory_node = findChild(root, {"trajectory"});
  const JsonValue * header_node = nullptr;
  if (trajectory_node) {
    header_node = trajectory_node->getMember("header");
  }
  if (!header_node) {
    header_node = root.getMember("header");
  }
  if (header_node) {
    result.trajectory.header = parseHeader(header_node);
  }

  const JsonValue * points_node = nullptr;
  if (trajectory_node) {
    points_node = trajectory_node->getMember("points");
  }
  if (!points_node) {
    points_node = root.getMember("points");
  }
  if (points_node && points_node->type == JsonType::Array) {
    for (const auto & point_value : points_node->array) {
      result.trajectory.points.push_back(parseTrajectoryPoint(point_value));
    }
  }
  return result;
}

std::int64_t toNanoseconds(const motion_velocity_planner::TimeStamp & stamp)
{
  return stamp.sec * static_cast<std::int64_t>(1000000000LL) + stamp.nanosec;
}

template <typename T>
std::vector<jsonl_utils::TimeStampedFrame<T>> loadSequence(
  const std::filesystem::path & jsonl_path,
  const std::function<T(const JsonValue &)> & converter,
  const std::function<motion_velocity_planner::TimeStamp(
    const JsonValue &, const JsonValue &, const T &)> & stamp_extractor)
{
  std::vector<jsonl_utils::TimeStampedFrame<T>> frames;
  std::ifstream ifs(jsonl_path);
  if (!ifs) {
    throw std::runtime_error("Unable to read " + jsonl_path.string());
  }
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const JsonValue root = JsonParser(line).parse();
    const JsonValue * message_node = root.getMember("message");
    const JsonValue & message_root = message_node ? *message_node : root;
    const T data = converter(message_root);
    const motion_velocity_planner::TimeStamp stamp =
      stamp_extractor(root, message_root, data);
    frames.push_back({stamp, std::move(data)});
  }
  std::sort(frames.begin(), frames.end(), [](const auto & a, const auto & b) {
    return toNanoseconds(a.stamp) < toNanoseconds(b.stamp);
  });
  return frames;
}

}  // namespace

std::vector<jsonl_utils::TimeStampedFrame<motion_velocity_planner::LocalizationKinematicState>>
jsonl_utils::loadLocalizationFrames(const std::filesystem::path & jsonl_path)
{
  return loadSequence<motion_velocity_planner::LocalizationKinematicState>(
    jsonl_path,
    [](const JsonValue & payload) { return parseLocalizationStateJson(payload); },
    [](const JsonValue &, const JsonValue &, const motion_velocity_planner::LocalizationKinematicState & state) {
      return state.header.stamp;
    });
}

std::vector<jsonl_utils::TimeStampedFrame<std::vector<motion_velocity_planner::Object>>>
jsonl_utils::loadObjectFrames(const std::filesystem::path & jsonl_path)
{
  return loadSequence<std::vector<motion_velocity_planner::Object>>(
    jsonl_path,
    [](const JsonValue & root) {
      std::vector<motion_velocity_planner::Object> objects;
      if (const auto * array = root.getMember("objects")) {
        if (array->type == JsonType::Array) {
          for (const auto & entry : array->array) {
            objects.push_back(parseObjectJson(entry));
          }
        }
      }
      return objects;
    },
    [](const JsonValue &, const JsonValue & payload, const std::vector<motion_velocity_planner::Object> &) {
      if (const auto * header = payload.getMember("header")) {
        return parseHeader(header).stamp;
      }
      return motion_velocity_planner::TimeStamp{};
    });
}

std::vector<jsonl_utils::TimeStampedFrame<motion_velocity_planner::PathOptimizerResult>>
jsonl_utils::loadPathOptimizerFrames(const std::filesystem::path & jsonl_path)
{
  return loadSequence<motion_velocity_planner::PathOptimizerResult>(
    jsonl_path,
    [](const JsonValue & payload) { return parsePathOptimizerJson(payload); },
    [](const JsonValue &, const JsonValue &, const motion_velocity_planner::PathOptimizerResult & result) {
      return result.trajectory.header.stamp;
    });
}

namespace
{

std::string escapeJsonString(std::string_view value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (char c : value) {
    switch (c) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\b':
        escaped += "\\b";
        break;
      case '\f':
        escaped += "\\f";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buffer[7];
          std::snprintf(buffer, sizeof(buffer), "\\u%04X", static_cast<unsigned char>(c));
          escaped += buffer;
        } else {
          escaped += c;
        }
        break;
    }
  }
  return escaped;
}

void writeHeaderObject(std::ostream & os, const motion_velocity_planner::Header & header)
{
  os << "\"header\":{";
  os << "\"stamp\":{\"sec\":" << header.stamp.sec << ",\"nanosec\":" << header.stamp.nanosec << "},";
  os << "\"frame_id\":\"" << escapeJsonString(header.frame_id) << "\"";
  os << "}";
}

void writePoseObject(std::ostream & os, const motion_velocity_planner::Pose & pose)
{
  os << "{\"position\":{";
  os << "\"x\":" << pose.position.x << ",\"y\":" << pose.position.y << ",\"z\":" << pose.position.z << "},";
  os << "\"orientation\":{";
  os << "\"x\":" << pose.orientation.x << ",\"y\":" << pose.orientation.y << ",\"z\":" << pose.orientation.z << ",\"w\":"
     << pose.orientation.w << "}}";
}

void writeTrajectoryPointObject(std::ostream & os, const motion_velocity_planner::TrajectoryPoint & point)
{
  os << "{";
  os << "\"time_from_start\":{";
  os << "\"sec\":" << point.time_from_start.sec << ",\"nanosec\":" << point.time_from_start.nanosec;
  os << "},";
  os << "\"pose\":";
  writePoseObject(os, point.pose);
  os << ",\"longitudinal_velocity_mps\":" << point.longitudinal_velocity_mps;
  os << ",\"lateral_velocity_mps\":" << point.lateral_velocity_mps;
  os << ",\"acceleration_mps2\":" << point.acceleration_mps2;
  os << ",\"heading_rate_rps\":" << point.heading_rate_rps;
  os << ",\"front_wheel_angle_rad\":" << point.front_wheel_angle_rad;
  os << ",\"rear_wheel_angle_rad\":" << point.rear_wheel_angle_rad;
  os << "}";
}

void writeTrajectoryObject(std::ostream & os, const motion_velocity_planner::Trajectory & trajectory)
{
  os << "{";
  writeHeaderObject(os, trajectory.header);
  os << ",\"points\":[";
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    if (i > 0) {
      os << ",";
    }
    writeTrajectoryPointObject(os, trajectory.points[i]);
  }
  os << "]}";
}

}  // namespace

std::string jsonl_utils::serializeMotionVelocityPlannerLine(
  const motion_velocity_planner::MotionVelocityPlannerOutput & output,
  const motion_velocity_planner::Header & header,
  std::size_t frame_index,
  int scenario_id)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(15);
  oss << "{";
  writeHeaderObject(oss, header);
  oss << ",\"trajectory\":";
  writeTrajectoryObject(oss, output.trajectory);
  oss << ",\"frame_index\":" << frame_index;
  oss << ",\"scenario_id\":" << scenario_id;
  oss << "}";
  return oss.str();
}
