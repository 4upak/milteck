#include "MissionProcessor.h"

#include <cstddef>
#include <string>

#include "Types.h"

namespace homework_08 {

MissionProcessor::MissionProcessor(IConfigLoader* loader, ITargetProvider* targets, IBallisticSolver* solver)
  : loader_(loader)
  , targets_(targets)
  , solver_(solver)
{
  if (loader_ == nullptr || targets_ == nullptr || solver_ == nullptr) {
    throw Homework08Error{"MissionProcessor: all three dependencies must be non-null"};
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
  return current_idx_ < targets_->targets().size();
}

DropPoint MissionProcessor::step()
{
  if (!initialized_) {
    throw Homework08Error{"MissionProcessor::step() called before init()"};
  }
  if (!hasNext()) {
    throw Homework08Error{"MissionProcessor::step() called past the last target"};
  }

  const std::vector<Target>& items = targets_->targets();
  const Target& target = items[current_idx_];
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
    throw Homework08Error{"MissionProcessor::changeSolver: solver must be non-null"};
  }
  solver_ = solver;
}

std::size_t MissionProcessor::currentIndex() const
{
  return current_idx_;
}

const MissionConfig& MissionProcessor::config() const
{
  if (!initialized_) {
    throw Homework08Error{"MissionProcessor::config() before init()"};
  }
  return loader_->getConfig();
}

const AmmoParams& MissionProcessor::ammo() const
{
  if (!initialized_) {
    throw Homework08Error{"MissionProcessor::ammo() before init()"};
  }
  return loader_->getAmmoParams();
}

}  // namespace homework_08
