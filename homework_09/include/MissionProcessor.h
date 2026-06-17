#pragma once

// MissionProcessor — Strategy coordinator для homework_09.
//
// На відміну від homework_08, процесор володіє компонентами через
// std::unique_ptr: loader, provider і solver живуть рівно стільки, скільки
// живе MissionProcessor. Заміна solver-а також передає власність.

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "Types.h"

namespace homework_09 {

class MissionProcessor {
public:
  MissionProcessor(
    std::unique_ptr<IConfigLoader> loader, std::unique_ptr<ITargetProvider> targets, std::unique_ptr<IBallisticSolver> solver);

  void init(const std::string& configSource);

  [[nodiscard]] bool hasNext() const;
  DropPoint step();
  void reset();
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);

  [[nodiscard]] std::size_t currentIndex() const;
  [[nodiscard]] const MissionConfig& config() const;
  [[nodiscard]] const AmmoParams& ammo() const;
  [[nodiscard]] const std::vector<Target>& targets() const;

private:
  std::unique_ptr<IConfigLoader> loader_;
  std::unique_ptr<ITargetProvider> targets_;
  std::unique_ptr<IBallisticSolver> solver_;
  std::size_t current_idx_ = 0;
  bool initialized_ = false;
};

}  // namespace homework_09
