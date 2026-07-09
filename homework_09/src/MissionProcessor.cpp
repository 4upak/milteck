#include "MissionProcessor.h"

#include <cmath>
#include <cstddef>
#include <algorithm>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include "Types.h"

namespace homework_09 {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kDefaultAcceleration = 10.0;
constexpr double kDefaultAngularSpeed = std::numbers::pi / 2.0;
constexpr double kDefaultTimeStep = 1.0;
constexpr double kDefaultTurnThreshold = 0.2;
constexpr double kDropReachTolerance = 2.0;

[[nodiscard]] double positive_or_default(double value, double fallback)
{
  return value > kEpsilon ? value : fallback;
}

[[nodiscard]] Coord predictedTargetPosition(const Target& target, double time)
{
  return Coord{target.pos.x + target.velocity.x * time, target.pos.y + target.velocity.y * time};
}

[[nodiscard]] double distance(Coord a, Coord b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

[[nodiscard]] double distancePointToSegment(Coord point, Coord a, Coord b)
{
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double apx = point.x - a.x;
  const double apy = point.y - a.y;
  const double ab_len2 = abx * abx + aby * aby;
  if (ab_len2 <= kEpsilon) {
    return distance(point, a);
  }

  const double t = std::clamp((apx * abx + apy * aby) / ab_len2, 0.0, 1.0);
  return distance(point, Coord{a.x + abx * t, a.y + aby * t});
}

[[nodiscard]] double directionTo(Coord from, Coord to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

}  // namespace

MissionProcessor::MissionProcessor(
  std::unique_ptr<IConfigLoader> loader, std::unique_ptr<ITargetProvider> targets, std::unique_ptr<IBallisticSolver> solver)
  : loader_(std::move(loader))
  , targets_(std::move(targets))
  , solver_(std::move(solver))
  , state_(std::make_unique<StateStopped>())
{
  if (loader_ == nullptr || targets_ == nullptr || solver_ == nullptr) {
    throw Homework09Error{"MissionProcessor: all three dependencies must be non-null"};
  }
}

void MissionProcessor::init(const std::string& configSource)
{
  loader_->load(configSource);

  const MissionConfig& cfg = loader_->getConfig();
  drone_ = DroneContext{};
  drone_.position = cfg.drone_pos;
  drone_.speed = 0.0;
  drone_.direction = 0.0;
  drone_.desired_direction = 0.0;
  drone_.target_direction = 0.0;
  drone_.turn_remaining = 0.0;
  drone_.cfg.attack_speed = positive_or_default(cfg.attack_speed, 1.0);
  drone_.cfg.acceleration = kDefaultAcceleration;
  drone_.cfg.angular_speed = kDefaultAngularSpeed;
  drone_.cfg.sim_time_step = kDefaultTimeStep;
  drone_.cfg.turn_threshold = kDefaultTurnThreshold;

  state_ = std::make_unique<StateStopped>();
  last_drop_ = DropPoint{};
  current_idx_ = 0;
  current_time_ = 0.0;
  steps_done_ = 0;
  initialized_ = true;
}

bool MissionProcessor::hasNext() const
{
  return initialized_ && current_idx_ < targets_->targets().size();
}

DropPoint MissionProcessor::step()
{
  if (!initialized_) {
    throw Homework09Error{"MissionProcessor::step() called before init()"};
  }
  if (!hasNext()) {
    throw Homework09Error{"MissionProcessor::step() called past the last target"};
  }
  if (steps_done_ >= max_steps_) {
    throw Homework09Error{"MissionProcessor::step() exceeded simulation step limit"};
  }

  const std::vector<Target>& items = targets_->targets();
  const Target& target = items[current_idx_];
  const MissionConfig& cfg = loader_->getConfig();
  const AmmoParams& ammo = loader_->getAmmoParams();
  const double dt = positive_or_default(drone_.cfg.sim_time_step, kDefaultTimeStep);

  const Coord position_before = drone_.position;
  const Coord target_now = predictedTargetPosition(target, current_time_);
  last_drop_ = solver_->solve(drone_.position, target_now, cfg.altitude, drone_.cfg.attack_speed, ammo);
  drone_.desired_direction = directionTo(drone_.position, last_drop_.pos);

  std::unique_ptr<IDroneState> next = state_->execute(drone_);
  if (next != nullptr) {
    state_ = std::move(next);
  }

  current_time_ += dt;
  ++steps_done_;

  const double reach_tolerance = std::max(kDropReachTolerance, drone_.cfg.attack_speed * dt);
  if (distancePointToSegment(last_drop_.pos, position_before, drone_.position) <= reach_tolerance) {
    ++current_idx_;
    state_ = std::make_unique<StateStopped>();
    drone_.speed = 0.0;
  }

  return last_drop_;
}

void MissionProcessor::reset()
{
  if (!initialized_) {
    throw Homework09Error{"MissionProcessor::reset() before init()"};
  }

  const MissionConfig& cfg = loader_->getConfig();
  drone_.position = cfg.drone_pos;
  drone_.speed = 0.0;
  drone_.direction = 0.0;
  drone_.desired_direction = 0.0;
  drone_.target_direction = 0.0;
  drone_.turn_remaining = 0.0;
  state_ = std::make_unique<StateStopped>();
  last_drop_ = DropPoint{};
  current_idx_ = 0;
  current_time_ = 0.0;
  steps_done_ = 0;
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> solver)
{
  if (solver == nullptr) {
    throw Homework09Error{"MissionProcessor::changeSolver: solver must be non-null"};
  }
  solver_ = std::move(solver);
}

std::size_t MissionProcessor::currentIndex() const
{
  return current_idx_;
}

double MissionProcessor::currentTime() const
{
  return current_time_;
}

Coord MissionProcessor::dronePosition() const
{
  return drone_.position;
}

const char* MissionProcessor::stateName() const
{
  return state_ != nullptr ? state_->name() : "<none>";
}

const MissionConfig& MissionProcessor::config() const
{
  if (!initialized_) {
    throw Homework09Error{"MissionProcessor::config() before init()"};
  }
  return loader_->getConfig();
}

const AmmoParams& MissionProcessor::ammo() const
{
  if (!initialized_) {
    throw Homework09Error{"MissionProcessor::ammo() before init()"};
  }
  return loader_->getAmmoParams();
}

const std::vector<Target>& MissionProcessor::targets() const
{
  return targets_->targets();
}

}  // namespace homework_09
