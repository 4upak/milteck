#include "mission_processor.hpp"

#include <string>

#include "types.hpp"

namespace homework_07 {

MissionProcessor::MissionProcessor(IConfigLoader* loader, ITargetProvider* targets, IBallisticSolver* solver)
  : loader_(loader)
  , targets_(targets)
  , solver_(solver)
{
  if (loader_ == nullptr || targets_ == nullptr || solver_ == nullptr) {
    throw Homework07Error{"MissionProcessor: all three dependencies must be non-null"};
  }
}

void MissionProcessor::init(const std::string& configSource)
{
  loader_->load(configSource);
  current_idx_ = 0;
  initialized_ = true;
}

bool MissionProcessor::hasNext() const
{
  if (!initialized_) {
    return false;
  }
  return current_idx_ < targets_->getTargetCount();
}

DropPoint MissionProcessor::step()
{
  if (!initialized_) {
    throw Homework07Error{"MissionProcessor::step() called before init()"};
  }
  if (!hasNext()) {
    throw Homework07Error{"MissionProcessor::step() called past the last target"};
  }

  const Target target = targets_->getTarget(current_idx_);
  const MissionConfig& cfg = loader_->getConfig();
  const AmmoParams& ammo = loader_->getAmmoParams();

  const DropPoint drop = solver_->solve(cfg.drone_pos, target.pos, cfg.altitude, cfg.attack_speed, ammo);
  ++current_idx_;
  return drop;
}

void MissionProcessor::reset()
{
  current_idx_ = 0;
}

void MissionProcessor::changeSolver(IBallisticSolver* solver)
{
  if (solver == nullptr) {
    throw Homework07Error{"MissionProcessor::changeSolver: solver must be non-null"};
  }
  solver_ = solver;
}

int MissionProcessor::currentIndex() const
{
  return current_idx_;
}

const MissionConfig& MissionProcessor::config() const
{
  if (!initialized_) {
    throw Homework07Error{"MissionProcessor::config() before init()"};
  }
  return loader_->getConfig();
}

const AmmoParams& MissionProcessor::ammo() const
{
  if (!initialized_) {
    throw Homework07Error{"MissionProcessor::ammo() before init()"};
  }
  return loader_->getAmmoParams();
}

}  // namespace homework_07
