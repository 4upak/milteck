#include "ballistics.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

void print_usage()
{
  std::cerr << "usage: ballistics_cli <input_path> [ammo_table_path]\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc != 2 && argc != 3) {
    print_usage();
    return 1;
  }

  try {
    // argv indexes are guarded by the argc check above.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const std::filesystem::path input_path{argv[1]};
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const std::filesystem::path ammo_path = argc == 3 ? std::filesystem::path{argv[2]} : std::filesystem::path{};

    const homework_06::AmmoTable ammo_table = argc == 3 ? homework_06::load_ammo_table(ammo_path) : homework_06::default_ammo_table();
    const std::vector<homework_06::BallisticsInput> scenarios = homework_06::load_inputs(input_path);

    std::cout << std::fixed << std::setprecision(3);
    for (std::size_t index = 0; index < scenarios.size(); ++index) {
      const homework_06::BallisticsInput& input = scenarios[index];
      const homework_06::AmmoInfo ammo = homework_06::lookup_ammo(ammo_table, input.ammo_name);
      const homework_06::DropSolution solution = homework_06::compute_drop_solution(input, ammo);

      std::cout << "case " << (index + 1) << " fire_x " << solution.fire_x << " fire_y " << solution.fire_y << " time_of_flight "
                << solution.time_of_flight << " horizontal_distance " << solution.horizontal_distance << " needs_maneuver "
                << (solution.needs_maneuver ? "yes" : "no") << '\n';
    }
  }
  catch (const homework_06::BallisticsError& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  catch (const std::exception& error) {
    std::cerr << "unexpected error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
