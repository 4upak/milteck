#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "interfaces/IBallisticSolver.h"
#include "Types.h"

namespace homework_09 {

struct BallisticTableResult {
  double time_of_flight = 0.0;
  double horizontal_distance = 0.0;
};

struct BallisticTable {
  std::vector<double> axisZ0;
  std::vector<double> axisV0;
  std::vector<double> axisM;
  std::vector<double> axisD;
  std::vector<double> axisL;
  std::vector<BallisticTableResult> data;

  [[nodiscard]] std::size_t index(std::size_t iz, std::size_t iv, std::size_t im, std::size_t id, std::size_t il) const;
  [[nodiscard]] const BallisticTableResult& at(std::size_t iz, std::size_t iv, std::size_t im, std::size_t id, std::size_t il) const;

  void load(const std::string& path);
  [[nodiscard]] BallisticTableResult lookup(double z0, double v0, double mass, double drag, double lift) const;
};

class TableSolver : public IBallisticSolver {
public:
  explicit TableSolver(const std::string& table_path);
  explicit TableSolver(BallisticTable table);

  [[nodiscard]] DropPoint solve(
    Coord drone_pos, Coord target_pos, double altitude, double attack_speed, const AmmoParams& ammo) const override;

  [[nodiscard]] const BallisticTable& table() const;

private:
  BallisticTable table_;
};

}  // namespace homework_09
