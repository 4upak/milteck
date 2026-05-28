#include "json_target_provider.hpp"

#include <exception>
#include <fstream>
#include <string>

#include "json.hpp"
#include "types.hpp"

namespace homework_07 {

namespace {

Coord read_coord(const nlohmann::json& node, const std::string& field)
{
  if (!node.contains(field) || !node.at(field).is_object()) {
    throw Homework07Error{"target field '" + field + "' must be an object"};
  }
  const nlohmann::json& obj = node.at(field);
  if (!obj.contains("x") || !obj.contains("y") || !obj.at("x").is_number() || !obj.at("y").is_number()) {
    throw Homework07Error{"coordinate '" + field + "' must contain numeric x and y"};
  }
  return Coord{obj.at("x").get<double>(), obj.at("y").get<double>()};
}

}  // namespace

JsonTargetProvider::JsonTargetProvider(const std::string& path)
{
  std::ifstream input{path};
  if (!input) {
    throw Homework07Error{"cannot open targets file: " + path};
  }

  nlohmann::json data;
  try {
    input >> data;
  }
  catch (const std::exception& parse_error) {
    throw Homework07Error{"invalid JSON in " + path + ": " + parse_error.what()};
  }

  if (!data.contains("targets") || !data.at("targets").is_array()) {
    throw Homework07Error{"targets file must contain a 'targets' array"};
  }
  const nlohmann::json& items = data.at("targets");
  if (items.empty()) {
    throw Homework07Error{"targets array is empty in " + path};
  }

  targets_.reserve(items.size());
  for (std::size_t i = 0; i < items.size(); ++i) {
    const nlohmann::json& entry = items.at(i);
    if (!entry.is_object()) {
      throw Homework07Error{"target #" + std::to_string(i) + " must be an object"};
    }

    Target target;
    target.pos = read_coord(entry, "pos");
    target.velocity = read_coord(entry, "velocity");
    if (entry.contains("name") && entry.at("name").is_string()) {
      target.name = entry.at("name").get<std::string>();
    }
    else {
      target.name = "target_" + std::to_string(i);
    }
    targets_.push_back(std::move(target));
  }
}

int JsonTargetProvider::getTargetCount() const
{
  return static_cast<int>(targets_.size());
}

Target JsonTargetProvider::getTarget(int index) const
{
  if (index < 0 || index >= static_cast<int>(targets_.size())) {
    throw Homework07Error{"target index out of range: " + std::to_string(index)};
  }
  return targets_[static_cast<std::size_t>(index)];
}

}  // namespace homework_07
