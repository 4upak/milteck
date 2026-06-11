#include "file_config_loader.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <string>

#include "json.hpp"
#include "types.hpp"

namespace homework_07 {

namespace {

nlohmann::json read_json_file(const std::filesystem::path& path)
{
  std::ifstream input{path};
  if (!input) {
    throw Homework07Error{"cannot open " + path.string()};
  }
  nlohmann::json data;
  try {
    input >> data;
  }
  catch (const std::exception& parse_error) {
    throw Homework07Error{"invalid JSON in " + path.string() + ": " + parse_error.what()};
  }
  return data;
}

MissionConfig parse_mission_config(const nlohmann::json& data, const std::filesystem::path& path)
{
  if (!data.contains("drone") || !data.at("drone").is_object()) {
    throw Homework07Error{path.string() + " must contain a 'drone' object"};
  }
  const nlohmann::json& drone = data.at("drone");
  if (!drone.contains("position") || !drone.at("position").is_object()) {
    throw Homework07Error{path.string() + " must contain drone.position object"};
  }

  MissionConfig config;
  const nlohmann::json& pos = drone.at("position");
  if (!pos.contains("x") || !pos.contains("y") || !pos.at("x").is_number() || !pos.at("y").is_number()) {
    throw Homework07Error{path.string() + " drone.position must have numeric x and y"};
  }
  config.drone_pos = Coord{pos.at("x").get<double>(), pos.at("y").get<double>()};

  if (!drone.contains("altitude") || !drone.at("altitude").is_number()) {
    throw Homework07Error{path.string() + " must contain drone.altitude"};
  }
  config.altitude = drone.at("altitude").get<double>();

  if (!drone.contains("attackSpeed") || !drone.at("attackSpeed").is_number()) {
    throw Homework07Error{path.string() + " must contain drone.attackSpeed"};
  }
  config.attack_speed = drone.at("attackSpeed").get<double>();

  if (!data.contains("ammo") || !data.at("ammo").is_string()) {
    throw Homework07Error{path.string() + " must contain ammo name (string)"};
  }
  config.ammo_name = data.at("ammo").get<std::string>();
  return config;
}

AmmoParams find_ammo(const nlohmann::json& data, const std::string& wanted, const std::filesystem::path& path)
{
  if (!data.is_array() || data.empty()) {
    throw Homework07Error{path.string() + " must be a non-empty array"};
  }

  for (std::size_t i = 0; i < data.size(); ++i) {
    const nlohmann::json& entry = data.at(i);
    if (!entry.is_object() || !entry.contains("name") || !entry.at("name").is_string()) {
      throw Homework07Error{"ammo entry #" + std::to_string(i) + " missing 'name'"};
    }
    if (entry.at("name").get<std::string>() != wanted) {
      continue;
    }

    AmmoParams ammo;
    ammo.name = wanted;
    ammo.mass = entry.at("mass").get<double>();
    ammo.drag = entry.at("drag").get<double>();
    ammo.lift = entry.at("lift").get<double>();
    if (ammo.mass <= 0.0) {
      throw Homework07Error{"ammo '" + wanted + "' has non-positive mass"};
    }
    return ammo;
  }

  throw Homework07Error{"unknown ammo type: " + wanted};
}

}  // namespace

void FileConfigLoader::load(const std::string& source)
{
  const std::filesystem::path base{source};
  const std::filesystem::path config_path = base / "config.json";
  const std::filesystem::path ammo_path = base / "ammo.json";

  config_ = parse_mission_config(read_json_file(config_path), config_path);
  ammo_ = find_ammo(read_json_file(ammo_path), config_.ammo_name, ammo_path);
  loaded_ = true;
}

const MissionConfig& FileConfigLoader::getConfig() const
{
  if (!loaded_) {
    throw Homework07Error{"FileConfigLoader: getConfig() called before load()"};
  }
  return config_;
}

const AmmoParams& FileConfigLoader::getAmmoParams() const
{
  if (!loaded_) {
    throw Homework07Error{"FileConfigLoader: getAmmoParams() called before load()"};
  }
  return ammo_;
}

}  // namespace homework_07
