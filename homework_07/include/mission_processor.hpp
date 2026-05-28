#pragma once

// MissionProcessor — реалізація патерну Стратегія для нашого refactor-у.
//
// Тримає три невласницьких (non-owning) вказівники на інтерфейси і
// делегує реальну роботу їм. Сам процесор не знає, чи це AnalyticalSolver
// чи TableSolver, чи це JsonTargetProvider чи TestTargetProvider —
// підмінюємо одну реалізацію іншою без зміни цього класу.

#include <string>

#include "i_ballistic_solver.hpp"
#include "i_config_loader.hpp"
#include "i_target_provider.hpp"
#include "types.hpp"

namespace homework_07 {

class MissionProcessor {
public:
  MissionProcessor(IConfigLoader* loader, ITargetProvider* targets, IBallisticSolver* solver);

  // Через loader зчитує конфіг і параметри боєприпасу, ставить ітератор
  // на початок. Після цього hasNext()/step() можна викликати.
  void init(const std::string& configSource);

  // Чи є ще необроблені цілі.
  [[nodiscard]] bool hasNext() const;

  // Обчислює DropPoint для поточної цілі, посуває ітератор уперед.
  // Кидає Homework07Error, якщо init() не виконувався або цілі скінчилися.
  DropPoint step();

  // Повертає ітератор на початок без перезавантаження конфігу.
  void reset();

  // Підмінює solver на льоту (це і є патерн Стратегія в дії).
  void changeSolver(IBallisticSolver* solver);

  // Допоміжне для CLI/тестів: поточний індекс цілі (ще не оброблений)
  // і метадані поточної конфігурації.
  [[nodiscard]] int currentIndex() const;
  [[nodiscard]] const MissionConfig& config() const;
  [[nodiscard]] const AmmoParams& ammo() const;

private:
  IConfigLoader* loader_;
  ITargetProvider* targets_;
  IBallisticSolver* solver_;
  int current_idx_ = 0;
  bool initialized_ = false;
};

}  // namespace homework_07
