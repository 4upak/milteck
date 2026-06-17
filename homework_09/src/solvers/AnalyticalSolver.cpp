#include "solvers/AnalyticalSolver.h"

#include <array>
#include <cmath>
#include <numbers>

#include "Types.h"

namespace homework_09 {

namespace {

constexpr double kGravity = 9.81;
constexpr double kEpsilon = 1e-9;
constexpr double kCubicCoeffEpsilon = 1e-9;
constexpr double kAcosArgUpperBound = 1.0;
constexpr double kAcosArgLowerBound = -1.0;
constexpr std::size_t kCubicRootCount = 3;

// Розв'язує a*t^3 + b*t^2 + c = 0 методом Кардано і повертає найменший
// додатний корінь. Кидає Homework09Error, якщо рівняння не має додатного
// розв'язку — це означає, що балістична задача в цих умовах нерозв'язна.
//
// На відміну від homework_07, три корені тримаються у std::array<double, 3>
// і перебираються range-based for-ом — це ілюстрація лекції про std::array
// фіксованого розміру (нуль overhead, дані на стеку).
double solve_cardano_time(double cubic_a, double cubic_b, double cubic_c)
{
  if (std::fabs(cubic_a) < kCubicCoeffEpsilon) {
    throw Homework09Error{"flight time equation degenerates (a ~ 0)"};
  }

  const double depressed_p = -(cubic_b * cubic_b) / (3.0 * cubic_a * cubic_a);
  const double depressed_q = (2.0 * cubic_b * cubic_b * cubic_b) / (27.0 * cubic_a * cubic_a * cubic_a) + cubic_c / cubic_a;

  if (depressed_p >= 0.0) {
    throw Homework09Error{"flight time equation has no real positive root (p >= 0)"};
  }

  const double acos_arg = (3.0 * depressed_q / (2.0 * depressed_p)) * std::sqrt(-3.0 / depressed_p);
  if (acos_arg < kAcosArgLowerBound || acos_arg > kAcosArgUpperBound) {
    throw Homework09Error{"flight time equation has complex roots (|acos arg| > 1)"};
  }

  const double phi = std::acos(acos_arg);
  const double root_base = 2.0 * std::sqrt(-depressed_p / 3.0);
  const double offset = cubic_b / (3.0 * cubic_a);
  const double two_pi = 2.0 * std::numbers::pi;

  const std::array<double, kCubicRootCount> roots = {
    root_base * std::cos(phi / 3.0) - offset,
    root_base * std::cos((phi + two_pi) / 3.0) - offset,
    root_base * std::cos((phi + 2.0 * two_pi) / 3.0) - offset,
  };

  // Беремо найменший додатний корінь — це і є фізично коректний час польоту.
  // Range-based for + std::array — стандартний STL-патерн замість трьох
  // окремих іменованих змінних.
  double smallest_positive = -1.0;
  for (const double candidate : roots) {
    if (candidate > kEpsilon && (smallest_positive < 0.0 || candidate < smallest_positive)) {
      smallest_positive = candidate;
    }
  }

  if (smallest_positive < 0.0) {
    throw Homework09Error{"flight time equation has no positive root"};
  }
  return smallest_positive;
}

// Горизонтальна дальність боєприпасу за час t. П'ять членів полінома —
// та сама форма, що використовувалася в ДЗ1.
double calc_horizontal_distance(double elapsed_time, const AmmoParams& ammo, double attack_speed)
{
  const double mass = ammo.mass;
  const double drag = ammo.drag;
  const double lift = ammo.lift;
  const double initial_speed = attack_speed;

  const double term1 = initial_speed * elapsed_time;
  const double term2 = -(elapsed_time * elapsed_time * drag * initial_speed) / (2.0 * mass);
  const double term3 = (elapsed_time * elapsed_time * elapsed_time *
                        (6.0 * drag * kGravity * lift * mass - 6.0 * drag * drag * (lift * lift - 1.0) * initial_speed)) /
                       (36.0 * mass * mass);
  const double term4 =
    (std::pow(elapsed_time, 4.0) * (-6.0 * drag * drag * kGravity * lift * (1.0 + lift * lift + lift * lift * lift * lift) * mass +
                                    3.0 * drag * drag * drag * lift * lift * (1.0 + lift * lift) * initial_speed +
                                    6.0 * drag * drag * drag * lift * lift * lift * lift * (1.0 + lift * lift) * initial_speed)) /
    (36.0 * std::pow(1.0 + lift * lift, 2.0) * mass * mass * mass);
  const double term5 = (std::pow(elapsed_time, 5.0) * (3.0 * drag * drag * drag * kGravity * lift * lift * lift * mass -
                                                       3.0 * std::pow(drag, 4.0) * lift * lift * (1.0 + lift * lift) * initial_speed)) /
                       (36.0 * (1.0 + lift * lift) * std::pow(mass, 4.0));

  return term1 + term2 + term3 + term4 + term5;
}

}  // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
DropPoint AnalyticalSolver::solve(Coord drone_pos, Coord target_pos, double altitude, double attack_speed, const AmmoParams& ammo) const
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

  const double cubic_a = ammo.drag * kGravity * ammo.mass - 2.0 * ammo.drag * ammo.drag * ammo.lift * attack_speed;
  const double cubic_b = -3.0 * kGravity * ammo.mass * ammo.mass + 3.0 * ammo.drag * ammo.lift * ammo.mass * attack_speed;
  const double cubic_c = 6.0 * ammo.mass * ammo.mass * altitude;

  const double time_of_flight = solve_cardano_time(cubic_a, cubic_b, cubic_c);
  const double horizontal_distance = calc_horizontal_distance(time_of_flight, ammo, attack_speed);
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
  DropPoint drop;
  drop.pos.x = drone_pos.x + delta_x * ratio;
  drop.pos.y = drone_pos.y + delta_y * ratio;
  drop.time_of_flight = time_of_flight;
  drop.horizontal_distance = horizontal_distance;
  return drop;
}

}  // namespace homework_09
