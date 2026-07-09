#pragma once

// Інтерфейс балістичного солвера.
//
// Контракт: за позицією дрона, позицією цілі, висотою та параметрами
// боєприпасу повертає точку, куди дрон має полетіти, щоб скинути снаряд.
// Реалізації можуть бути аналітичні або табличні.

#include "Types.h"

namespace homework_10 {

class IBallisticSolver {
public:
  virtual ~IBallisticSolver() = default;

  IBallisticSolver() = default;
  IBallisticSolver(const IBallisticSolver&) = delete;
  IBallisticSolver& operator=(const IBallisticSolver&) = delete;
  IBallisticSolver(IBallisticSolver&&) = delete;
  IBallisticSolver& operator=(IBallisticSolver&&) = delete;

  [[nodiscard]] virtual DropPoint solve(
    Coord drone_pos, Coord target_pos, double altitude, double attack_speed, const AmmoParams& ammo) const = 0;
};

}  // namespace homework_10
