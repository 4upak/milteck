#pragma once

// MissionProcessor — реалізація патерну Стратегія для homework_08.
//
// Тримає три невласницьких (non-owning) вказівники на інтерфейси і
// делегує реальну роботу їм. Сам процесор не знає, чи це AnalyticalSolver
// чи TableSolver, чи це JsonTargetProvider чи TestTargetProvider —
// підмінюємо одну реалізацію іншою без зміни цього класу.

#include <cstddef>
#include <string>

#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "Types.h"

namespace homework_08 {

class MissionProcessor {
public:
  MissionProcessor(IConfigLoader* loader, ITargetProvider* targets, IBallisticSolver* solver);

  // Через loader зчитує конфіг і параметри боєприпасу, ставить ітератор
  // на початок. Після цього hasNext()/step() можна викликати.
  void init(const std::string& configSource);

  // Чи є ще необроблені цілі.
  [[nodiscard]] bool hasNext() const;

  // Обчислює DropPoint для поточної цілі, посуває ітератор уперед.
  // Кидає Homework08Error, якщо init() не виконувався або цілі скінчилися.
  DropPoint step();

  // Повертає ітератор на початок без перезавантаження конфігу.
  void reset();

  // Підмінює solver на льоту (це і є патерн Стратегія в дії).
  void changeSolver(IBallisticSolver* solver);

  // Допоміжне для CLI/тестів: поточний індекс цілі (ще не оброблений)
  // і метадані поточної конфігурації.
  [[nodiscard]] std::size_t currentIndex() const;
  [[nodiscard]] const MissionConfig& config() const;
  [[nodiscard]] const AmmoParams& ammo() const;

private:
  IConfigLoader* loader_;
  ITargetProvider* targets_;
  IBallisticSolver* solver_;
  std::size_t current_idx_ = 0;
  bool initialized_ = false;
};

}  // namespace homework_08
