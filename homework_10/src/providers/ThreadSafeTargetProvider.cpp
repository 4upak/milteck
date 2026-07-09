#include "providers/ThreadSafeTargetProvider.h"

#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <thread>
#include <utility>

#include "json.hpp"

namespace homework_10 {
namespace {

Coord readCoordObject(const nlohmann::json& node)
{
  if (!node.is_object() || !node.contains("x") || !node.contains("y") || !node.at("x").is_number() || !node.at("y").is_number()) {
    throw Homework10Error{"coordinate must contain numeric x and y"};
  }
  return Coord{node.at("x").get<double>(), node.at("y").get<double>()};
}

Coord readCoordField(const nlohmann::json& node, const std::string& field)
{
  if (!node.contains(field)) {
    throw Homework10Error{"target missing coordinate field: " + field};
  }
  return readCoordObject(node.at(field));
}

}  // namespace

ThreadSafeTargetProvider::ThreadSafeTargetProvider(std::string path, double arrayTimeStep, double timeScale)
  : array_time_step_(arrayTimeStep > 0.0 ? arrayTimeStep : 0.05)
  , time_scale_(timeScale > 0.0 ? timeScale : 1.0)
{
  load(path);
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider()
{
  stop();
}

bool ThreadSafeTargetProvider::isThreadReady() const
{
  return ready_.load();
}

void ThreadSafeTargetProvider::run()
{
  ready_.store(true);
  while (!stop_requested_.load() && !started_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  while (!stop_requested_.load()) {
    advanceOneStep();
    std::this_thread::sleep_for(std::chrono::duration<double>(array_time_step_ / time_scale_));
  }
}

void ThreadSafeTargetProvider::start()
{
  started_.store(true);
}

void ThreadSafeTargetProvider::stop()
{
  stop_requested_.store(true);
}

std::size_t ThreadSafeTargetProvider::getTargetCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

Target ThreadSafeTargetProvider::getTarget(std::size_t index) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= targets_.size()) {
    throw Homework10Error{"target index out of range"};
  }
  return targets_[index].snapshot;
}

std::vector<Target> ThreadSafeTargetProvider::targets() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Target> result;
  result.reserve(targets_.size());
  for (const MovingTarget& target : targets_) {
    result.push_back(target.snapshot);
  }
  return result;
}

Target ThreadSafeTargetProvider::findByName(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = name_to_index_.find(name);
  if (it == name_to_index_.end()) {
    throw Homework10Error{"unknown target: " + name};
  }
  return targets_[it->second].snapshot;
}

void ThreadSafeTargetProvider::load(const std::string& path)
{
  std::ifstream input{path};
  if (!input) {
    throw Homework10Error{"cannot open targets file: " + path};
  }

  nlohmann::json data;
  try {
    input >> data;
  }
  catch (const std::exception& parse_error) {
    throw Homework10Error{"invalid JSON in " + path + ": " + parse_error.what()};
  }

  if (!data.contains("targets") || !data.at("targets").is_array() || data.at("targets").empty()) {
    throw Homework10Error{"targets file must contain a non-empty targets array"};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  targets_.clear();
  name_to_index_.clear();

  std::size_t index = 0;
  for (const nlohmann::json& item : data.at("targets")) {
    if (!item.is_object()) {
      throw Homework10Error{"target must be an object"};
    }

    MovingTarget moving;
    moving.name = item.contains("name") && item.at("name").is_string() ? item.at("name").get<std::string>() : "target_" + std::to_string(index);

    if (item.contains("trajectory") && item.at("trajectory").is_array()) {
      for (const nlohmann::json& point : item.at("trajectory")) {
        moving.trajectory.push_back(readCoordObject(point));
      }
    }
    else {
      const Coord pos = readCoordField(item, "pos");
      const Coord velocity = item.contains("velocity") ? readCoordField(item, "velocity") : Coord{};
      moving.trajectory.push_back(pos);
      moving.trajectory.push_back(pos + velocity * array_time_step_);
    }

    if (moving.trajectory.empty()) {
      throw Homework10Error{"target trajectory is empty"};
    }
    if (moving.trajectory.size() == 1) {
      moving.trajectory.push_back(moving.trajectory.front());
    }

    const Coord next = moving.trajectory[1 % moving.trajectory.size()];
    moving.snapshot = Target{moving.trajectory.front(), (next - moving.trajectory.front()) * (1.0 / array_time_step_), moving.name};
    name_to_index_[moving.name] = targets_.size();
    targets_.push_back(std::move(moving));
    ++index;
  }
}

void ThreadSafeTargetProvider::advanceOneStep()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (MovingTarget& target : targets_) {
    const std::size_t next_index = (target.current + 1) % target.trajectory.size();
    const Coord current_pos = target.trajectory[target.current];
    const Coord next_pos = target.trajectory[next_index];
    target.snapshot.pos = next_pos;
    target.snapshot.velocity = (next_pos - current_pos) * (1.0 / array_time_step_);
    target.snapshot.name = target.name;
    target.current = next_index;
  }
}

}  // namespace homework_10
