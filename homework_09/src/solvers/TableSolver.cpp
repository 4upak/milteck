#include "solvers/TableSolver.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "Types.h"

namespace homework_09 {
namespace {

constexpr double kEpsilon = 1e-9;

struct Interp {
  std::size_t lo = 0;
  double frac = 0.0;
};

BallisticTableResult lerp(const BallisticTableResult& a, const BallisticTableResult& b, double t)
{
  return BallisticTableResult{
    a.time_of_flight + (b.time_of_flight - a.time_of_flight) * t,
    a.horizontal_distance + (b.horizontal_distance - a.horizontal_distance) * t,
  };
}

void validate_axis(const std::vector<double>& axis, const std::string& name)
{
  if (axis.size() < 2) {
    throw Homework09Error{"ballistic table axis " + name + " must contain at least two values"};
  }
  if (!std::is_sorted(axis.begin(), axis.end())) {
    throw Homework09Error{"ballistic table axis " + name + " must be sorted"};
  }
  for (std::size_t i = 1; i < axis.size(); ++i) {
    if (std::fabs(axis[i] - axis[i - 1]) <= kEpsilon) {
      throw Homework09Error{"ballistic table axis " + name + " contains duplicate values"};
    }
  }
}

Interp find_interp(double value, const std::vector<double>& axis)
{
  if (value <= axis.front()) {
    return Interp{0, 0.0};
  }
  if (value >= axis.back()) {
    return Interp{axis.size() - 2, 1.0};
  }

  const auto upper = std::lower_bound(axis.begin(), axis.end(), value);
  const std::size_t lo = static_cast<std::size_t>(std::distance(axis.begin(), upper) - 1);
  const double frac = (value - axis[lo]) / (axis[lo + 1] - axis[lo]);
  return Interp{lo, frac};
}

std::vector<double> read_axis(std::ifstream& input, std::size_t count, const std::string& name)
{
  std::vector<double> axis(count);
  for (double& value : axis) {
    if (!(input >> value)) {
      throw Homework09Error{"cannot read ballistic table axis " + name};
    }
  }
  validate_axis(axis, name);
  return axis;
}

}  // namespace

std::size_t BallisticTable::index(std::size_t iz, std::size_t iv, std::size_t im, std::size_t id, std::size_t il) const
{
  return ((((iz * axisV0.size() + iv) * axisM.size() + im) * axisD.size() + id) * axisL.size() + il);
}

const BallisticTableResult& BallisticTable::at(
  std::size_t iz, std::size_t iv, std::size_t im, std::size_t id, std::size_t il) const
{
  const std::size_t flat = index(iz, iv, im, id, il);
  if (flat >= data.size()) {
    throw Homework09Error{"ballistic table index out of range"};
  }
  return data[flat];
}

void BallisticTable::load(const std::string& path)
{
  std::ifstream input{path};
  if (!input) {
    throw Homework09Error{"cannot open ballistic table: " + path};
  }

  std::size_t nZ = 0;
  std::size_t nV = 0;
  std::size_t nM = 0;
  std::size_t nD = 0;
  std::size_t nL = 0;
  if (!(input >> nZ >> nV >> nM >> nD >> nL)) {
    throw Homework09Error{"cannot read ballistic table dimensions: " + path};
  }

  axisZ0 = read_axis(input, nZ, "Z0");
  axisV0 = read_axis(input, nV, "V0");
  axisM = read_axis(input, nM, "m");
  axisD = read_axis(input, nD, "d");
  axisL = read_axis(input, nL, "l");

  const std::size_t total = nZ * nV * nM * nD * nL;
  data.resize(total);
  for (BallisticTableResult& result : data) {
    if (!(input >> result.time_of_flight >> result.horizontal_distance)) {
      throw Homework09Error{"not enough ballistic table result rows: " + path};
    }
    if (result.time_of_flight <= 0.0 || result.horizontal_distance <= 0.0) {
      throw Homework09Error{"ballistic table results must be positive"};
    }
  }
}

BallisticTableResult BallisticTable::lookup(double z0, double v0, double mass, double drag, double lift) const
{
  validate_axis(axisZ0, "Z0");
  validate_axis(axisV0, "V0");
  validate_axis(axisM, "m");
  validate_axis(axisD, "d");
  validate_axis(axisL, "l");

  const std::size_t expected = axisZ0.size() * axisV0.size() * axisM.size() * axisD.size() * axisL.size();
  if (data.size() != expected) {
    throw Homework09Error{"ballistic table data size does not match axes"};
  }

  const Interp iz = find_interp(z0, axisZ0);
  const Interp iv = find_interp(v0, axisV0);
  const Interp im = find_interp(mass, axisM);
  const Interp id = find_interp(drag, axisD);
  const Interp il = find_interp(lift, axisL);

  BallisticTableResult after_l[16];
  for (std::size_t a = 0; a < 2; ++a) {
    for (std::size_t b = 0; b < 2; ++b) {
      for (std::size_t c = 0; c < 2; ++c) {
        for (std::size_t e = 0; e < 2; ++e) {
          const BallisticTableResult& lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
          const BallisticTableResult& hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
          after_l[a * 8 + b * 4 + c * 2 + e] = lerp(lo, hi, il.frac);
        }
      }
    }
  }

  BallisticTableResult after_d[8];
  for (std::size_t a = 0; a < 2; ++a) {
    for (std::size_t b = 0; b < 2; ++b) {
      for (std::size_t c = 0; c < 2; ++c) {
        after_d[a * 4 + b * 2 + c] = lerp(after_l[a * 8 + b * 4 + c * 2], after_l[a * 8 + b * 4 + c * 2 + 1], id.frac);
      }
    }
  }

  BallisticTableResult after_m[4];
  for (std::size_t a = 0; a < 2; ++a) {
    for (std::size_t b = 0; b < 2; ++b) {
      after_m[a * 2 + b] = lerp(after_d[a * 4 + b * 2], after_d[a * 4 + b * 2 + 1], im.frac);
    }
  }

  BallisticTableResult after_v[2];
  for (std::size_t a = 0; a < 2; ++a) {
    after_v[a] = lerp(after_m[a * 2], after_m[a * 2 + 1], iv.frac);
  }

  return lerp(after_v[0], after_v[1], iz.frac);
}

TableSolver::TableSolver(const std::string& table_path)
{
  table_.load(table_path);
}

TableSolver::TableSolver(BallisticTable table)
  : table_(std::move(table))
{
}

DropPoint TableSolver::solve(Coord drone_pos, Coord target_pos, double altitude, double attack_speed, const AmmoParams& ammo) const
{
  if (altitude <= 0.0) {
    throw Homework09Error{"altitude must be positive"};
  }
  if (attack_speed < 0.0) {
    throw Homework09Error{"attack speed must be non-negative"};
  }
  if (ammo.mass <= 0.0) {
    throw Homework09Error{"ammo mass must be positive"};
  }

  const BallisticTableResult table_result = table_.lookup(altitude, attack_speed, ammo.mass, ammo.drag, ammo.lift);
  const double horizontal_distance = table_result.horizontal_distance;
  if (!std::isfinite(horizontal_distance) || horizontal_distance <= kEpsilon) {
    throw Homework09Error{"horizontal distance must be positive and finite"};
  }

  const double delta_x = target_pos.x - drone_pos.x;
  const double delta_y = target_pos.y - drone_pos.y;
  const double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
  if (distance <= kEpsilon) {
    throw Homework09Error{"distance to target must be positive"};
  }

  const double ratio = (distance - horizontal_distance) / distance;
  return DropPoint{
    Coord{drone_pos.x + delta_x * ratio, drone_pos.y + delta_y * ratio},
    table_result.time_of_flight,
    horizontal_distance,
  };
}

const BallisticTable& TableSolver::table() const
{
  return table_;
}

}  // namespace homework_09
