#pragma once

#include "interfaces/IBallisticSolver.h"
#include "Types.h"

namespace homework_10 {

// Аналітичний солвер: бере формулу з ДЗ1 (Кардано + полінома горизонтальної
// дальності). Повертає DropPoint, обчислений геометрично.
class AnalyticalSolver : public IBallisticSolver {
public:
  [[nodiscard]] DropPoint solve(
    Coord drone_pos, Coord target_pos, double altitude, double attack_speed, const AmmoParams& ammo) const override;
};

}  // namespace homework_10
