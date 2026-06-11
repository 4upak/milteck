#include "ballistics.hpp"

#include <cctype>
#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <utility>

namespace homework_06 {

namespace {

constexpr double kGravity = 9.81;
constexpr double kEpsilon = 1e-9;

// Numeric thresholds used by both the Cardano solver and the geometry checks.
// Naming them keeps clang-tidy magic-number lints quiet without sprinkling
// NOLINT around the file.
constexpr double kCubicCoeffEpsilon = 1e-9;
constexpr double kAcosArgUpperBound = 1.0;
constexpr double kAcosArgLowerBound = -1.0;

// Solves a*t^3 + b*t^2 + c = 0 via Cardano's formula and returns the smallest
// positive real root. Prefers t3 to match the convention from homework_01;
// falls back to t2 and t1 if needed. The returned value is positive on
// success; on failure throws BallisticsError so callers cannot silently use a
// bogus zero.
double solve_cardano_time(double cubic_a, double cubic_b, double cubic_c)
{
  if (std::fabs(cubic_a) < kCubicCoeffEpsilon) {
    throw BallisticsError{"flight time equation degenerates (a ~ 0)"};
  }

  const double depressed_p = -(cubic_b * cubic_b) / (3.0 * cubic_a * cubic_a);
  const double depressed_q = (2.0 * cubic_b * cubic_b * cubic_b) / (27.0 * cubic_a * cubic_a * cubic_a) + cubic_c / cubic_a;

  if (depressed_p >= 0.0) {
    throw BallisticsError{"flight time equation has no real positive root (p >= 0)"};
  }

  const double acos_arg = (3.0 * depressed_q / (2.0 * depressed_p)) * std::sqrt(-3.0 / depressed_p);
  if (acos_arg < kAcosArgLowerBound || acos_arg > kAcosArgUpperBound) {
    throw BallisticsError{"flight time equation has complex roots (|acos arg| > 1)"};
  }

  const double phi = std::acos(acos_arg);
  const double root_base = 2.0 * std::sqrt(-depressed_p / 3.0);
  const double offset = cubic_b / (3.0 * cubic_a);
  const double two_pi = 2.0 * std::numbers::pi;

  const double root_1 = root_base * std::cos(phi / 3.0) - offset;
  const double root_2 = root_base * std::cos((phi + two_pi) / 3.0) - offset;
  const double root_3 = root_base * std::cos((phi + 2.0 * two_pi) / 3.0) - offset;

  if (root_3 > kEpsilon) {
    return root_3;
  }
  if (root_2 > kEpsilon) {
    return root_2;
  }
  if (root_1 > kEpsilon) {
    return root_1;
  }

  throw BallisticsError{"flight time equation has no positive root"};
}

// Horizontal range of the munition after `t` seconds. The five terms are the
// Taylor-style polynomial used in homework_01 and intentionally kept verbose:
// the goal is parity with the existing reference output, not aerodynamic
// elegance.
double calc_horizontal_distance(double elapsed_time, const AmmoInfo& ammo, double attack_speed)
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

// Trims ASCII whitespace from both sides of a string view.
std::string_view trim(std::string_view text)
{
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())) != 0) {
    text.remove_prefix(1);
  }
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())) != 0) {
    text.remove_suffix(1);
  }
  return text;
}

}  // namespace

AmmoTable default_ammo_table()
{
  // Mirrors the table from dz1/ammo_data.txt that ships with the course
  // materials. Kept in sync with data/ammo_data.txt so tests do not need any
  // disk access for the canonical entries.
  return AmmoTable{
    {"VOG-17", {0.35, 0.07, 0.0}},
    {"M67", {0.60, 0.10, 0.0}},
    {"RKG-3", {1.20, 0.10, 0.0}},
    {"GLIDING-VOG", {0.45, 0.10, 1.0}},
    {"GLIDING-RKG", {1.40, 0.10, 1.0}},
  };
}

AmmoTable load_ammo_table(const std::filesystem::path& path)
{
  std::ifstream input{path};
  if (!input) {
    throw BallisticsError{"cannot open ammo table: " + path.string()};
  }

  AmmoTable table;
  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const std::string_view trimmed = trim(line);
    if (trimmed.empty()) {
      continue;
    }

    std::istringstream parser{std::string{trimmed}};
    std::string name;
    AmmoInfo info;
    if (!(parser >> name >> info.mass >> info.drag >> info.lift)) {
      throw BallisticsError{"malformed ammo row at line " + std::to_string(line_number)};
    }
    if (info.mass <= 0.0) {
      throw BallisticsError{"non-positive mass at line " + std::to_string(line_number)};
    }

    table.insert_or_assign(std::move(name), info);
  }

  if (table.empty()) {
    throw BallisticsError{"ammo table is empty: " + path.string()};
  }

  return table;
}

AmmoInfo lookup_ammo(const AmmoTable& table, std::string_view name)
{
  const auto ammo_it = table.find(std::string{name});
  if (ammo_it == table.end()) {
    throw BallisticsError{"unknown ammo type: " + std::string{name}};
  }
  return ammo_it->second;
}

std::vector<BallisticsInput> load_inputs(const std::filesystem::path& path)
{
  std::ifstream input{path};
  if (!input) {
    throw BallisticsError{"cannot open input file: " + path.string()};
  }

  std::vector<BallisticsInput> scenarios;
  BallisticsInput current;
  while (input >> current.drone_x >> current.drone_y >> current.drone_z >> current.target_x >> current.target_y >> current.attack_speed >>
         current.acceleration_path >> current.ammo_name) {
    scenarios.push_back(current);
  }

  if (!input.eof()) {
    throw BallisticsError{"malformed input file (incomplete scenario): " + path.string()};
  }

  if (scenarios.empty()) {
    throw BallisticsError{"input file contains no scenarios: " + path.string()};
  }

  return scenarios;
}

DropSolution compute_drop_solution(const BallisticsInput& input, const AmmoInfo& ammo)
{
  if (ammo.mass <= 0.0) {
    throw BallisticsError{"ammo mass must be positive"};
  }
  if (input.drone_z <= 0.0) {
    throw BallisticsError{"drone altitude must be positive"};
  }
  if (input.attack_speed < 0.0) {
    throw BallisticsError{"attack speed must be non-negative"};
  }
  if (input.acceleration_path < 0.0) {
    throw BallisticsError{"acceleration path must be non-negative"};
  }

  const double cubic_a = ammo.drag * kGravity * ammo.mass - 2.0 * ammo.drag * ammo.drag * ammo.lift * input.attack_speed;
  const double cubic_b = -3.0 * kGravity * ammo.mass * ammo.mass + 3.0 * ammo.drag * ammo.lift * ammo.mass * input.attack_speed;
  const double cubic_c = 6.0 * ammo.mass * ammo.mass * input.drone_z;

  const double time_of_flight = solve_cardano_time(cubic_a, cubic_b, cubic_c);
  const double horizontal_distance = calc_horizontal_distance(time_of_flight, ammo, input.attack_speed);
  if (!std::isfinite(horizontal_distance) || horizontal_distance <= kEpsilon) {
    throw BallisticsError{"horizontal distance must be positive and finite"};
  }

  const double target_dx = input.target_x - input.drone_x;
  const double target_dy = input.target_y - input.drone_y;
  const double distance_to_target = std::sqrt(target_dx * target_dx + target_dy * target_dy);
  if (distance_to_target <= kEpsilon) {
    throw BallisticsError{"distance to target must be positive"};
  }

  DropSolution solution;
  solution.time_of_flight = time_of_flight;
  solution.horizontal_distance = horizontal_distance;
  solution.distance_to_target = distance_to_target;
  solution.intermediate_x = input.drone_x;
  solution.intermediate_y = input.drone_y;
  solution.needs_maneuver = horizontal_distance + input.acceleration_path > distance_to_target;

  if (solution.needs_maneuver) {
    solution.intermediate_x =
      input.target_x - (input.target_x - input.drone_x) * (horizontal_distance + input.acceleration_path) / distance_to_target;
    solution.intermediate_y =
      input.target_y - (input.target_y - input.drone_y) * (horizontal_distance + input.acceleration_path) / distance_to_target;
  }

  const double attack_dx = input.target_x - solution.intermediate_x;
  const double attack_dy = input.target_y - solution.intermediate_y;
  const double attack_distance = std::sqrt(attack_dx * attack_dx + attack_dy * attack_dy);
  if (attack_distance <= kEpsilon) {
    throw BallisticsError{"attack start distance must be positive"};
  }

  const double ratio = (attack_distance - horizontal_distance) / attack_distance;
  solution.fire_x = solution.intermediate_x + attack_dx * ratio;
  solution.fire_y = solution.intermediate_y + attack_dy * ratio;

  return solution;
}

}  // namespace homework_06