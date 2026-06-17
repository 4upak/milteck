#include "MissionProcessor.h"

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Types.h"

namespace homework_09 {

MissionProcessor::MissionProcessor(
  std::unique_ptr<IConfigLoader> loader, std::unique_ptr<ITargetProvider> targets, std::unique_ptr<IBallisticSolver> solver)
  : loader_(std::move(loader))
  , targets_(std::move(targets))
  , solver_(std::move(solver))
{
  if (loader_ == nullptr || targets_ == nullptr || solver_ == nullptr) {
    throw Homework09Error{"MissionProcessor: all three dependencies must be non-null"};
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
  if (!initialized_) {
    throw Homework09Error{"MissionProcessor::reset() before init()"};
  }
  current_idx_ = 0;
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
