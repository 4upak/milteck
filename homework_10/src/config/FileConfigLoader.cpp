#include "config/FileConfigLoader.h"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>

#include "json.hpp"
#include "Types.h"

namespace homework_10 {

namespace {

nlohmann::json read_json_file(const std::filesystem::path& path)
{
  std::ifstream input{path};
  if (!input) {
    throw Homework10Error{"cannot open " + path.string()};
  }
  nlohmann::json data;
  try {
    input >> data;
  }
  catch (const std::exception& parse_error) {
    throw Homework10Error{"invalid JSON in " + path.string() + ": " + parse_error.what()};
  }
  return data;
}

MissionConfig parse_mission_config(const nlohmann::json& data, const std::filesystem::path& path)
{
  if (!data.contains("drone") || !data.at("drone").is_object()) {
    throw Homework10Error{path.string() + " must contain a 'drone' object"};
  }
  const nlohmann::json& drone = data.at("drone");
  if (!drone.contains("position") || !drone.at("position").is_object()) {
    throw Homework10Error{path.string() + " must contain drone.position object"};
  }

  MissionConfig config;
  const nlohmann::json& pos = drone.at("position");
  if (!pos.contains("x") || !pos.contains("y") || !pos.at("x").is_number() || !pos.at("y").is_number()) {
    throw Homework10Error{path.string() + " drone.position must have numeric x and y"};
  }
  config.drone_pos = Coord{pos.at("x").get<double>(), pos.at("y").get<double>()};

  if (!drone.contains("altitude") || !drone.at("altitude").is_number()) {
    throw Homework10Error{path.string() + " must contain drone.altitude"};
  }
  config.altitude = drone.at("altitude").get<double>();

  if (!drone.contains("attackSpeed") || !drone.at("attackSpeed").is_number()) {
    throw Homework10Error{path.string() + " must contain drone.attackSpeed"};
  }
  config.attack_speed = drone.at("attackSpeed").get<double>();

  if (!data.contains("ammo") || !data.at("ammo").is_string()) {
    throw Homework10Error{path.string() + " must contain ammo name (string)"};
  }
  config.ammo_name = data.at("ammo").get<std::string>();

  if (data.contains("simulation")) {
    const nlohmann::json& sim = data.at("simulation");
    if (!sim.is_object()) {
      throw Homework10Error{path.string() + " simulation must be an object"};
    }
    auto read_positive = [&](const char* key, double current) {
      if (!sim.contains(key)) {
        return current;
      }
      if (!sim.at(key).is_number()) {
        throw Homework10Error{path.string() + " simulation." + key + " must be numeric"};
      }
      const double value = sim.at(key).get<double>();
      return value > 0.0 ? value : current;
    };
    config.simulation.sim_time_step = read_positive("simTimeStep", config.simulation.sim_time_step);
    config.simulation.array_time_step = read_positive("arrayTimeStep", config.simulation.array_time_step);
    config.simulation.target_time_step = read_positive("targetTimeStep", config.simulation.array_time_step);
    config.simulation.physics_time_step = read_positive("physicsTimeStep", config.simulation.physics_time_step);
    config.simulation.time_scale = read_positive("timeScale", config.simulation.time_scale);
    config.simulation.max_mission_time = read_positive("maxMissionTime", config.simulation.max_mission_time);
  }
  return config;
}

// Завантажує усі рядки з ammo.json у std::unordered_map<std::string, AmmoParams>.
// Це канонічний рефакторинг "if/else ланцюжок / лінійний пошук → hash-таблиця"
// з лекції 16: name → AmmoParams за O(1).
std::unordered_map<std::string, AmmoParams> load_ammo_table(const nlohmann::json& data, const std::filesystem::path& path)
{
  if (!data.is_array() || data.empty()) {
    throw Homework10Error{path.string() + " must be a non-empty array"};
  }

  std::unordered_map<std::string, AmmoParams> table;
  table.reserve(data.size());

  std::size_t entry_index = 0;
  for (const nlohmann::json& entry : data) {
    if (!entry.is_object() || !entry.contains("name") || !entry.at("name").is_string()) {
      throw Homework10Error{"ammo entry #" + std::to_string(entry_index) + " missing 'name'"};
    }

    AmmoParams ammo;
    ammo.name = entry.at("name").get<std::string>();
    if (!entry.contains("mass") || !entry.contains("drag") || !entry.contains("lift")) {
      throw Homework10Error{"ammo '" + ammo.name + "' missing mass/drag/lift"};
    }
    if (!entry.at("mass").is_number() || !entry.at("drag").is_number() || !entry.at("lift").is_number()) {
      throw Homework10Error{"ammo '" + ammo.name + "' mass/drag/lift must be numeric"};
    }
    ammo.mass = entry.at("mass").get<double>();
    ammo.drag = entry.at("drag").get<double>();
    ammo.lift = entry.at("lift").get<double>();
    if (ammo.mass <= 0.0) {
      throw Homework10Error{"ammo '" + ammo.name + "' has non-positive mass"};
    }

    const auto [it, inserted] = table.try_emplace(ammo.name, std::move(ammo));
    if (!inserted) {
      throw Homework10Error{"duplicate ammo name in " + path.string() + ": " + it->first};
    }
    ++entry_index;
  }

  return table;
}

}  // namespace

void FileConfigLoader::load(const std::string& source)
{
  const std::filesystem::path base{source};
  const std::filesystem::path config_path = base / "config.json";
  const std::filesystem::path ammo_path = base / "ammo.json";

  config_ = parse_mission_config(read_json_file(config_path), config_path);
  ammo_table_ = load_ammo_table(read_json_file(ammo_path), ammo_path);

  const auto it = ammo_table_.find(config_.ammo_name);
  if (it == ammo_table_.end()) {
    throw Homework10Error{"unknown ammo type: " + config_.ammo_name};
  }
  selected_ammo_ = it->second;
  loaded_ = true;
}

const MissionConfig& FileConfigLoader::getConfig() const
{
  if (!loaded_) {
    throw Homework10Error{"FileConfigLoader: getConfig() called before load()"};
  }
  return config_;
}

const AmmoParams& FileConfigLoader::getAmmoParams() const
{
  if (!loaded_) {
    throw Homework10Error{"FileConfigLoader: getAmmoParams() called before load()"};
  }
  return selected_ammo_;
}

const std::unordered_map<std::string, AmmoParams>& FileConfigLoader::ammoTable() const
{
  if (!loaded_) {
    throw Homework10Error{"FileConfigLoader: ammoTable() called before load()"};
  }
  return ammo_table_;
}

}  // namespace homework_10
