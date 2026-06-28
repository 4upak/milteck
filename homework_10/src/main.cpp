#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "config/ComponentFactory.h"
#include "drone/DronePhysics.h"
#include "MissionProcessor.h"
#include "providers/ThreadSafeTargetProvider.h"
#include "Types.h"

namespace {

void print_usage()
{
  std::cerr << "usage: homework_10_cli <config_dir> [targets_path] [analytical|table] [table_path] [output_path]\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2 || argc > 6) {
    print_usage();
    return 1;
  }

  const std::string config_dir{argv[1]};  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const std::string targets_path = argc >= 3 ? std::string{argv[2]} : (config_dir + "/targets.json");
  const std::string solver_name = argc >= 4 ? std::string{argv[3]} : "analytical";
  const std::string output_path = argc >= 6 ? std::string{argv[5]} : "simulation.json";

  try {
    auto loader = homework_10::createLoader(homework_10::LoaderType::File);
    loader->load(config_dir);
    const homework_10::MissionConfig cfg = loader->getConfig();

    std::unique_ptr<homework_10::IBallisticSolver> solver;
    if (solver_name == "analytical") {
      solver = homework_10::createSolver(homework_10::SolverType::Analytical);
    }
    else if (solver_name == "table") {
      solver = argc >= 5 ? homework_10::createTableSolver(argv[4]) : homework_10::createSolver(homework_10::SolverType::Table);
    }
    else {
      throw homework_10::Homework10Error{"unknown solver: " + solver_name};
    }

    homework_10::ThreadSafeTargetProvider provider(targets_path, cfg.simulation.array_time_step, cfg.simulation.time_scale);
    homework_10::DronePhysics physics(cfg.drone_pos, cfg.attack_speed, cfg.simulation.physics_time_step, cfg.simulation.time_scale);
    homework_10::MissionProcessor mission(std::move(loader), provider, physics, std::move(solver), output_path);
    mission.init(config_dir);

    std::thread provider_thread(&homework_10::ThreadSafeTargetProvider::run, &provider);
    std::thread physics_thread(&homework_10::DronePhysics::run, &physics);
    std::thread mission_thread(&homework_10::MissionProcessor::run, &mission);

    while (!provider.isThreadReady() || !physics.isThreadReady() || !mission.isThreadReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    provider.start();
    physics.start();
    mission.start();

    mission_thread.join();
    physics.stop();
    provider.stop();
    physics_thread.join();
    provider_thread.join();

    std::cout << "simulation written to " << output_path << '\n';
  }
  catch (const homework_10::Homework10Error& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  catch (const std::exception& error) {
    std::cerr << "unexpected error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
