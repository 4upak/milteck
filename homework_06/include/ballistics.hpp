#pragma once

// Public API for the homework_06 ballistic drop solver.
//
// The header keeps domain types and pure functions decoupled from any I/O,
// so unit tests can call into the solver directly without spawning the CLI.

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace homework_06 {

// Physical parameters of a single munition entry.
struct AmmoInfo {
  double mass = 0.0;
  double drag = 0.0;
  double lift = 0.0;
};

using AmmoTable = std::unordered_map<std::string, AmmoInfo>;

// One scenario read from an input file: drone state, target and chosen ammo.
struct BallisticsInput {
  double drone_x = 0.0;
  double drone_y = 0.0;
  double drone_z = 0.0;
  double target_x = 0.0;
  double target_y = 0.0;
  double attack_speed = 0.0;
  double acceleration_path = 0.0;
  std::string ammo_name;
};

// Result of the ballistic computation for one scenario.
struct DropSolution {
  double time_of_flight = 0.0;
  double horizontal_distance = 0.0;
  double distance_to_target = 0.0;
  bool needs_maneuver = false;
  double intermediate_x = 0.0;
  double intermediate_y = 0.0;
  double fire_x = 0.0;
  double fire_y = 0.0;
};

// Controlled domain error: unknown ammo, malformed input, no positive flight
// time, etc. The CLI catches this to print a single line and exit non-zero,
// rather than crashing.
class BallisticsError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// Returns a small built-in ammo table that mirrors the dataset used in the
// original homework_01. Useful for tests that should not depend on file I/O.
AmmoTable default_ammo_table();

// Loads an ammo table from a whitespace-separated file: name mass drag lift.
// Throws BallisticsError on a missing file or malformed row.
AmmoTable load_ammo_table(const std::filesystem::path& path);

// Looks up ammo by name; throws BallisticsError when not present.
AmmoInfo lookup_ammo(const AmmoTable& table, std::string_view name);

// Loads one or more scenarios from a text file with eight whitespace-separated
// tokens per scenario, matching the homework_01 input format.
std::vector<BallisticsInput> load_inputs(const std::filesystem::path& path);

// Computes the drop solution for a single scenario.
// Throws BallisticsError on invalid geometry or unsolvable flight time.
DropSolution compute_drop_solution(const BallisticsInput& input, const AmmoInfo& ammo);

}  // namespace homework_06
