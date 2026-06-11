#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "config/ComponentFactory.h"
#include "MissionProcessor.h"
#include "Types.h"

namespace {

void print_usage()
{
  std::cerr << "usage: homework_08_cli <config_dir> [targets_path]\n"
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

  homework_08::IConfigLoader* loader = nullptr;
  homework_08::ITargetProvider* provider = nullptr;
  homework_08::IBallisticSolver* solver = nullptr;
  int exit_code = 0;

  try {
    loader = homework_08::createLoader(homework_08::LoaderType::File);
    provider = homework_08::createProvider(homework_08::ProviderType::Json, targets_path);
    solver = homework_08::createSolver(homework_08::SolverType::Analytical);

    homework_08::MissionProcessor mission(loader, provider, solver);
    mission.init(config_dir);

    const homework_08::MissionConfig& cfg = mission.config();
    const homework_08::AmmoParams& ammo = mission.ammo();
    const std::vector<homework_08::Target>& targets = provider->targets();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "mission start drone=(" << cfg.drone_pos.x << ", " << cfg.drone_pos.y << ")" << " altitude=" << cfg.altitude
              << " ammo=" << ammo.name << " targets=" << targets.size() << '\n';

    // Range-based for замість індексного while+hasNext()/getTarget(idx).
    // mission.step() усе одно потрібен — він рухає внутрішній індекс
    // Strategy-процесора і виконує балістику; зовнішній цикл просто
    // дає нам ім'я та позицію поточної цілі без зайвого виклику.
    std::size_t idx = 0;
    for (const homework_08::Target& target : targets) {
      const homework_08::DropPoint drop = mission.step();
      std::cout << "target " << idx << " '" << target.name << "'" << " pos=(" << target.pos.x << ", " << target.pos.y << ")" << " drop=("
                << drop.pos.x << ", " << drop.pos.y << ")" << " tof=" << drop.time_of_flight << " range=" << drop.horizontal_distance
                << '\n';
      ++idx;
    }
  }
  catch (const homework_08::Homework08Error& error) {
    std::cerr << "error: " << error.what() << '\n';
    exit_code = 1;
  }
  catch (const std::exception& error) {
    std::cerr << "unexpected error: " << error.what() << '\n';
    exit_code = 1;
  }

  // Власник створених фабрикою об'єктів — main(); прибираємо їх у зворотному
  // порядку. У продакшені тут жив би std::unique_ptr.
  // NOLINTBEGIN(cppcoreguidelines-owning-memory)
  delete solver;
  delete provider;
  delete loader;
  // NOLINTEND(cppcoreguidelines-owning-memory)
  return exit_code;
}
