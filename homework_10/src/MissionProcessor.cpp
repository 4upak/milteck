#include "MissionProcessor.h"

#include <algorithm>
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

constexpr double kEpsilon = 1e-9;
constexpr double kDefaultAcceleration = 10.0;
constexpr double kDefaultAngularSpeed = 1.5;
constexpr double kDefaultTurnThreshold = 0.2;
constexpr double kDropReachTolerance = 2.0;

[[nodiscard]] double positiveOrDefault(double value, double fallback)
{
  return value > kEpsilon ? value : fallback;
}

[[nodiscard]] double scalarSpeed(Coord speed)
{
  return std::hypot(speed.x, speed.y);
}

[[nodiscard]] double distance(Coord a, Coord b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

[[nodiscard]] double directionTo(Coord from, Coord to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

[[nodiscard]] DroneMode modeFromStateName(const char* name)
{
  const std::string state_name{name};
  if (state_name == "Accelerating") {
    return DroneMode::Accelerating;
  }
  if (state_name == "Decelerating") {
    return DroneMode::Decelerating;
  }
  if (state_name == "Turning") {
    return DroneMode::Turning;
  }
  if (state_name == "Moving") {
    return DroneMode::Moving;
  }
  return DroneMode::Stopped;
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
  , state_(std::make_unique<StateStopped>())
  , output_path_(std::move(outputPath))
{
  if (loader_ == nullptr || solver_ == nullptr) {
    throw Homework10Error{"MissionProcessor: loader and solver must be non-null"};
  }
}

void MissionProcessor::init(const std::string& configSource)
{
  loader_->load(configSource);
  state_ = std::make_unique<StateStopped>();
  current_idx_ = 0;
  initialized_ = true;
  stop_requested_.store(false);
  started_.store(false);
  ready_.store(false);
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
  const std::size_t max_steps = static_cast<std::size_t>(std::ceil(sim.max_mission_time / positiveOrDefault(sim.sim_time_step, 0.05)));
  log.reserve(max_steps + 1);

  for (std::size_t step_index = 0; step_index < max_steps && !stop_requested_.load() && hasNext(); ++step_index) {
    const Target target = targets_.getTarget(current_idx_);
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

    commandPhysicsFromState(telemetry, drop);
    log.push_back(SimulationStep{telemetry, current_idx_, drop, drop.pos, predicted});

    if (reachedDropPoint(telemetry, drop)) {
      ++current_idx_;
      state_ = std::make_unique<StateStopped>();
      physics_.pushCommand(DroneCommand{DroneMode::Stopped, kDefaultAngularSpeed, telemetry.direction, 0.0});
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(sim.sim_time_step / positiveOrDefault(sim.time_scale, 1.0)));
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
  commandPhysicsFromState(telemetry, drop);
  if (reachedDropPoint(telemetry, drop)) {
    ++current_idx_;
    state_ = std::make_unique<StateStopped>();
  }
  return drop;
}

void MissionProcessor::reset()
{
  if (!initialized_) {
    throw Homework10Error{"MissionProcessor::reset() before init()"};
  }
  current_idx_ = 0;
  state_ = std::make_unique<StateStopped>();
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

void MissionProcessor::commandPhysicsFromState(const DroneTelemetry& telemetry, const DropPoint& dropPoint)
{
  const MissionConfig& cfg = loader_->getConfig();
  const SimulationConfig& sim = cfg.simulation;
  const double desired_direction = directionTo(telemetry.pos, dropPoint.pos);

  DroneContext ctx;
  ctx.position = telemetry.pos;
  ctx.speed = scalarSpeed(telemetry.speed);
  ctx.direction = telemetry.direction;
  ctx.desired_direction = desired_direction;
  ctx.target_direction = desired_direction;
  ctx.cfg.attack_speed = positiveOrDefault(cfg.attack_speed, 1.0);
  ctx.cfg.acceleration = std::max(kDefaultAcceleration, cfg.attack_speed * 2.0);
  ctx.cfg.angular_speed = kDefaultAngularSpeed;
  ctx.cfg.sim_time_step = positiveOrDefault(sim.sim_time_step, 0.05);
  ctx.cfg.turn_threshold = kDefaultTurnThreshold;

  std::unique_ptr<IDroneState> next = state_->execute(ctx);
  if (next != nullptr) {
    state_ = std::move(next);
  }

  const DroneMode command_mode = modeFromStateName(state_->name());
  const double target_speed = (command_mode == DroneMode::Stopped) ? 0.0 : cfg.attack_speed;
  physics_.pushCommand(DroneCommand{command_mode, ctx.cfg.angular_speed, desired_direction, target_speed});
}

bool MissionProcessor::reachedDropPoint(const DroneTelemetry& telemetry, const DropPoint& dropPoint) const
{
  const SimulationConfig& sim = loader_->getConfig().simulation;
  const double tolerance = std::max(kDropReachTolerance, loader_->getConfig().attack_speed * positiveOrDefault(sim.sim_time_step, 0.05));
  return distance(telemetry.pos, dropPoint.pos) <= tolerance;
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
