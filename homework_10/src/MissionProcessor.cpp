#include "MissionProcessor.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "drone/DroneState.h"
#include "json.hpp"

namespace homework_10 {
namespace {

[[nodiscard]] double directionTo(Coord from, Coord to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

nlohmann::json coordToJson(Coord coord)
{
  return nlohmann::json{{"x", coord.x}, {"y", coord.y}};
}

}  // namespace

MissionProcessor::MissionProcessor(
  std::unique_ptr<IConfigLoader> loader,
  ITargetProvider& targets,
  DronePhysics& physics,
  std::unique_ptr<IBallisticSolver> solver,
  std::string outputPath)
  : loader_(std::move(loader))
  , targets_(targets)
  , physics_(physics)
  , solver_(std::move(solver))
  , output_path_(std::move(outputPath))
{
  if (loader_ == nullptr || solver_ == nullptr) {
    throw Homework10Error{"MissionProcessor: loader and solver must be non-null"};
  }
}

void MissionProcessor::init(const std::string& configSource)
{
  loader_->load(configSource);
  current_idx_ = 0;
  initialized_ = true;
}

bool MissionProcessor::isThreadReady() const
{
  return ready_.load();
}

void MissionProcessor::run()
{
  if (!initialized_) {
    throw Homework10Error{"MissionProcessor::run() before init()"};
  }

  ready_.store(true);
  while (!stop_requested_.load() && !started_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const SimulationConfig sim = loader_->getConfig().simulation;
  std::vector<SimulationStep> log;
  const std::size_t max_steps = static_cast<std::size_t>(std::ceil(sim.max_mission_time / sim.sim_time_step));
  log.reserve(max_steps + 1);

  for (std::size_t step_index = 0; step_index < max_steps && !stop_requested_.load(); ++step_index) {
    if (targets_.getTargetCount() == 0) {
      break;
    }

    const std::size_t target_index = step_index % targets_.getTargetCount();
    const Target target = targets_.getTarget(target_index);
    const DroneTelemetry telemetry = physics_.getTelemetry();

    Coord predicted{};
    DropPoint drop{};
    try {
      drop = planStep(telemetry, target, predicted);
    }
    catch (const Homework10Error&) {
      predicted = target.pos;
      drop.pos = target.pos;
    }

    const double desired_direction = directionTo(telemetry.pos, drop.pos);
    const double delta = normalizeAngle(desired_direction - telemetry.direction);
    const DroneMode mode = std::fabs(delta) > 0.05 ? DroneMode::Turning : DroneMode::Accelerating;
    physics_.pushCommand(DroneCommand{mode, 1.5, desired_direction, loader_->getConfig().attack_speed});

    log.push_back(SimulationStep{telemetry, target_index, drop, drop.pos, predicted});
    std::this_thread::sleep_for(std::chrono::duration<double>(sim.sim_time_step / sim.time_scale));
  }

  writeSimulationLog(log);
}

void MissionProcessor::start()
{
  started_.store(true);
}

void MissionProcessor::stop()
{
  stop_requested_.store(true);
}

bool MissionProcessor::hasNext() const
{
  return initialized_ && current_idx_ < targets_.getTargetCount();
}

DropPoint MissionProcessor::step()
{
  if (!initialized_) {
    throw Homework10Error{"MissionProcessor::step() called before init()"};
  }
  if (!hasNext()) {
    throw Homework10Error{"MissionProcessor::step() called past the last target"};
  }
  const DroneTelemetry telemetry = physics_.getTelemetry();
  const Target target = targets_.getTarget(current_idx_);
  Coord predicted{};
  const DropPoint drop = planStep(telemetry, target, predicted);
  ++current_idx_;
  return drop;
}

void MissionProcessor::reset()
{
  if (!initialized_) {
    throw Homework10Error{"MissionProcessor::reset() before init()"};
  }
  current_idx_ = 0;
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> solver)
{
  if (solver == nullptr) {
    throw Homework10Error{"MissionProcessor::changeSolver: solver must be non-null"};
  }
  solver_ = std::move(solver);
}

std::size_t MissionProcessor::currentIndex() const
{
  return current_idx_;
}

const MissionConfig& MissionProcessor::config() const
{
  if (!initialized_) {
    throw Homework10Error{"MissionProcessor::config() before init()"};
  }
  return loader_->getConfig();
}

const AmmoParams& MissionProcessor::ammo() const
{
  if (!initialized_) {
    throw Homework10Error{"MissionProcessor::ammo() before init()"};
  }
  return loader_->getAmmoParams();
}

DropPoint MissionProcessor::planStep(const DroneTelemetry& telemetry, const Target& target, Coord& predictedTarget) const
{
  const MissionConfig& cfg = loader_->getConfig();
  const AmmoParams& ammo_params = loader_->getAmmoParams();
  DropPoint preliminary = solver_->solve(telemetry.pos, target.pos, cfg.altitude, cfg.attack_speed, ammo_params);
  predictedTarget = target.pos + target.velocity * preliminary.time_of_flight;
  return solver_->solve(telemetry.pos, predictedTarget, cfg.altitude, cfg.attack_speed, ammo_params);
}

void MissionProcessor::writeSimulationLog(const std::vector<SimulationStep>& steps) const
{
  nlohmann::json root;
  root["steps"] = nlohmann::json::array();
  for (const SimulationStep& step : steps) {
    root["steps"].push_back({
      {"position", coordToJson(step.telemetry.pos)},
      {"direction", step.telemetry.direction},
      {"state", toString(step.telemetry.state)},
      {"targetIndex", step.targetIndex},
      {"dropPoint", coordToJson(step.dropPoint.pos)},
      {"aimPoint", coordToJson(step.aimPoint)},
      {"predictedTarget", coordToJson(step.predictedTarget)},
      {"timeSecSinceStart", step.telemetry.timeSecSinceStart},
    });
  }

  std::ofstream output{output_path_};
  if (!output) {
    throw Homework10Error{"cannot write simulation log: " + output_path_};
  }
  output << root.dump(2) << '\n';
}

}  // namespace homework_10
