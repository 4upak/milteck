#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "config/ComponentFactory.h"
#include "MissionProcessor.h"
#include "Types.h"

namespace {

void print_usage()
{
  std::cerr << "usage: homework_09_cli <config_dir> [targets_path] [analytical|table] [table_path]\n"
            << "  <config_dir>    directory with config.json and ammo.json\n"
            << "  [targets_path]  optional, defaults to <config_dir>/targets.json\n"
            << "  [solver]        optional, analytical or table; defaults to analytical\n"
            << "  [table_path]    optional path for table solver\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2 || argc > 5) {
    print_usage();
    return 1;
  }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const std::string config_dir{argv[1]};
  const std::string targets_path =
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    (argc >= 3) ? std::string{argv[2]} : (config_dir + "/targets.json");
  const std::string solver_name =
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    (argc >= 4) ? std::string{argv[3]} : "analytical";

  try {
    std::unique_ptr<homework_09::IBallisticSolver> solver;
    if (solver_name == "analytical") {
      solver = homework_09::createSolver(homework_09::SolverType::Analytical);
    } else if (solver_name == "table") {
      solver = (argc == 5)
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ? homework_09::createTableSolver(argv[4])
        : homework_09::createSolver(homework_09::SolverType::Table);
    } else {
      throw homework_09::Homework09Error{"unknown solver: " + solver_name};
    }

    homework_09::MissionProcessor mission(
      homework_09::createLoader(homework_09::LoaderType::File),
      homework_09::createProvider(homework_09::ProviderType::Json, targets_path),
      std::move(solver));
    mission.init(config_dir);

    const homework_09::MissionConfig& cfg = mission.config();
    const homework_09::AmmoParams& ammo = mission.ammo();
    const std::vector<homework_09::Target>& targets = mission.targets();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "mission start drone=(" << cfg.drone_pos.x << ", " << cfg.drone_pos.y << ")" << " altitude=" << cfg.altitude
              << " ammo=" << ammo.name << " solver=" << solver_name << " targets=" << targets.size() << '\n';

    std::size_t step = 0;
    while (mission.hasNext()) {
      const std::size_t target_idx = mission.currentIndex();
      const homework_09::Target& target = targets.at(target_idx);
      const homework_09::DropPoint drop = mission.step();
      const homework_09::Coord drone = mission.dronePosition();
      std::cout << "step " << step << " t=" << mission.currentTime() << " state=" << mission.stateName() << " target " << target_idx
                << " '" << target.name << "'" << " drone=(" << drone.x << ", " << drone.y << ")" << " drop=(" << drop.pos.x << ", "
                << drop.pos.y << ")" << " tof=" << drop.time_of_flight << " range=" << drop.horizontal_distance << '\n';
      ++step;
    }
  }
  catch (const homework_09::Homework09Error& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  catch (const std::exception& error) {
    std::cerr << "unexpected error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
