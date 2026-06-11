#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

#include "factory.hpp"
#include "mission_processor.hpp"
#include "types.hpp"

namespace {

void print_usage()
{
  std::cerr << "usage: homework_07_cli <config_dir> [targets_path]\n"
            << "  <config_dir>    directory with config.json and ammo.json\n"
            << "  [targets_path]  optional, defaults to <config_dir>/targets.json\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc != 2 && argc != 3) {
    print_usage();
    return 1;
  }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const std::string config_dir{argv[1]};
  const std::string targets_path =
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    (argc == 3) ? std::string{argv[2]} : (config_dir + "/targets.json");

  // Створюємо компоненти через фабрику. Викликаючий код тут оперує лише
  // інтерфейсами і не знає, які саме реалізації стоять за ними.
  homework_07::IConfigLoader* loader = nullptr;
  homework_07::ITargetProvider* provider = nullptr;
  homework_07::IBallisticSolver* solver = nullptr;
  int exit_code = 0;

  try {
    loader = homework_07::createLoader(homework_07::LoaderType::File);
    provider = homework_07::createProvider(homework_07::ProviderType::Json, targets_path);
    solver = homework_07::createSolver(homework_07::SolverType::Analytical);

    homework_07::MissionProcessor mission(loader, provider, solver);
    mission.init(config_dir);

    const homework_07::MissionConfig& cfg = mission.config();
    const homework_07::AmmoParams& ammo = mission.ammo();
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "mission start drone=(" << cfg.drone_pos.x << ", " << cfg.drone_pos.y << ")" << " altitude=" << cfg.altitude
              << " ammo=" << ammo.name << " targets=" << provider->getTargetCount() << '\n';

    while (mission.hasNext()) {
      const int idx = mission.currentIndex();
      const homework_07::Target target = provider->getTarget(idx);
      const homework_07::DropPoint drop = mission.step();
      std::cout << "target " << idx << " '" << target.name << "'" << " pos=(" << target.pos.x << ", " << target.pos.y << ")" << " drop=("
                << drop.pos.x << ", " << drop.pos.y << ")" << " tof=" << drop.time_of_flight << " range=" << drop.horizontal_distance
                << '\n';
    }
  }
  catch (const homework_07::Homework07Error& error) {
    std::cerr << "error: " << error.what() << '\n';
    exit_code = 1;
  }
  catch (const std::exception& error) {
    std::cerr << "unexpected error: " << error.what() << '\n';
    exit_code = 1;
  }

  // Власник створених фабрикою об'єктів — main(); прибираємо їх у зворотному
  // порядку. У продакшені тут жив би std::unique_ptr; навчально показуємо
  // явну схему raw pointer + delete з лекції 14.
  // NOLINTBEGIN(cppcoreguidelines-owning-memory)
  delete solver;
  delete provider;
  delete loader;
  // NOLINTEND(cppcoreguidelines-owning-memory)
  return exit_code;
}
