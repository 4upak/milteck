#include "providers/JsonTargetProvider.h"

#include <cstddef>
#include <exception>
#include <fstream>
#include <string>

#include "json.hpp"
#include "Types.h"

namespace homework_08 {

namespace {

Coord read_coord(const nlohmann::json& node, const std::string& field)
{
  if (!node.contains(field) || !node.at(field).is_object()) {
    throw Homework08Error{"target field '" + field + "' must be an object"};
  }
  const nlohmann::json& obj = node.at(field);
  if (!obj.contains("x") || !obj.contains("y") || !obj.at("x").is_number() || !obj.at("y").is_number()) {
    throw Homework08Error{"coordinate '" + field + "' must contain numeric x and y"};
  }
  return Coord{obj.at("x").get<double>(), obj.at("y").get<double>()};
}

}  // namespace

JsonTargetProvider::JsonTargetProvider(const std::string& path)
{
  std::ifstream input{path};
  if (!input) {
    throw Homework08Error{"cannot open targets file: " + path};
  }

  nlohmann::json data;
  try {
    input >> data;
  }
  catch (const std::exception& parse_error) {
    throw Homework08Error{"invalid JSON in " + path + ": " + parse_error.what()};
  }

  if (!data.contains("targets") || !data.at("targets").is_array()) {
    throw Homework08Error{"targets file must contain a 'targets' array"};
  }
  const nlohmann::json& items = data.at("targets");
  if (items.empty()) {
    throw Homework08Error{"targets array is empty in " + path};
  }

  targets_.reserve(items.size());

  // Range-based for замість for(size_t i = 0; ...) — заодно отримуємо порядковий
  // номер автоматично через targets_.size() перед push_back.
  std::size_t json_index = 0;
  for (const nlohmann::json& entry : items) {
    if (!entry.is_object()) {
      throw Homework08Error{"target #" + std::to_string(json_index) + " must be an object"};
    }

    Target target;
    target.pos = read_coord(entry, "pos");
    target.velocity = read_coord(entry, "velocity");
    if (entry.contains("name") && entry.at("name").is_string()) {
      target.name = entry.at("name").get<std::string>();
    }
    else {
      target.name = "target_" + std::to_string(json_index);
    }

    const std::size_t inserted_index = targets_.size();
    name_to_index_[target.name] = inserted_index;
    targets_.push_back(std::move(target));
    ++json_index;
  }
}

const std::vector<Target>& JsonTargetProvider::targets() const
{
  return targets_;
}

const Target* JsonTargetProvider::findByName(const std::string& name) const
{
  // O(1) lookup через std::unordered_map — типовий приклад з лекції 16
  // (хеш-таблиця замість лінійного перебору / if-else ланцюжка).
  const auto it = name_to_index_.find(name);
  if (it == name_to_index_.end()) {
    return nullptr;
  }
  return &targets_[it->second];
}

}  // namespace homework_08
