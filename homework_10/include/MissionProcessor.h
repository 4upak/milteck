#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "drone/DronePhysics.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "Types.h"

namespace homework_10 {

class MissionProcessor {
public:
  MissionProcessor(
    std::unique_ptr<IConfigLoader> loader,
    ITargetProvider& targets,
    DronePhysics& physics,
    std::unique_ptr<IBallisticSolver> solver,
    std::string outputPath = "simulation.json");

  void init(const std::string& configSource);
  [[nodiscard]] bool isThreadReady() const;
  void run();
  void start();
  void stop();

  [[nodiscard]] bool hasNext() const;
  DropPoint step();
  void reset();
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);

  [[nodiscard]] std::size_t currentIndex() const;
  [[nodiscard]] const MissionConfig& config() const;
  [[nodiscard]] const AmmoParams& ammo() const;

private:
  struct SimulationStep {
    DroneTelemetry telemetry{};
    std::size_t targetIndex = 0;
    DropPoint dropPoint{};
    Coord aimPoint{};
    Coord predictedTarget{};
  };

  [[nodiscard]] DropPoint planStep(const DroneTelemetry& telemetry, const Target& target, Coord& predictedTarget) const;
  void writeSimulationLog(const std::vector<SimulationStep>& steps) const;

  std::unique_ptr<IConfigLoader> loader_;
  ITargetProvider& targets_;
  DronePhysics& physics_;
  std::unique_ptr<IBallisticSolver> solver_;
  std::string output_path_;
  std::size_t current_idx_ = 0;
  bool initialized_ = false;
  std::atomic<bool> ready_{false};
  std::atomic<bool> started_{false};
  std::atomic<bool> stop_requested_{false};
};

}  // namespace homework_10
