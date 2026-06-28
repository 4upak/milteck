#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "config/FileConfigLoader.h"
#include "drone/DronePhysics.h"
#include "interfaces/IBallisticSolver.h"
#include "MissionProcessor.h"
#include "providers/ThreadSafeTargetProvider.h"
#include "Types.h"
#include "utils/ThreadSafeQueue.h"

namespace homework_10 {
namespace {

class ConstantSolver : public IBallisticSolver {
public:
  DropPoint solve(Coord drone_pos, Coord target_pos, double, double, const AmmoParams&) const override
  {
    return DropPoint{Coord{(drone_pos.x + target_pos.x) / 2.0, (drone_pos.y + target_pos.y) / 2.0}, 0.25, 10.0};
  }
};

std::filesystem::path dataDir()
{
  return std::filesystem::path{HOMEWORK_10_DATA_DIR};
}

}  // namespace

TEST(ThreadSafeQueue, PushAndTryPopAreFifo)
{
  ThreadSafeQueue<int> queue;
  EXPECT_TRUE(queue.empty());
  queue.push(1);
  queue.push(2);
  ASSERT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.tryPop(), 1);
  EXPECT_EQ(queue.tryPop(), 2);
  EXPECT_FALSE(queue.tryPop().has_value());
}

TEST(FileConfigLoader, LoadsNewSimulationDefaultsAndOverrides)
{
  FileConfigLoader loader;
  loader.load(dataDir().string());
  const MissionConfig& cfg = loader.getConfig();
  EXPECT_DOUBLE_EQ(cfg.simulation.target_time_step, 0.05);
  EXPECT_DOUBLE_EQ(cfg.simulation.physics_time_step, 0.01);
  EXPECT_DOUBLE_EQ(cfg.simulation.time_scale, 10.0);
}

TEST(ThreadSafeTargetProvider, ReturnsSnapshotsAndAdvancesTrajectoryAfterStart)
{
  ThreadSafeTargetProvider provider((dataDir() / "targets.json").string(), 0.05, 100.0);
  EXPECT_FALSE(provider.isThreadReady());

  std::thread worker(&ThreadSafeTargetProvider::run, &provider);
  while (!provider.isThreadReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const Target before = provider.getTarget(0);
  provider.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  provider.stop();
  worker.join();

  const Target after = provider.getTarget(0);
  EXPECT_EQ(provider.getTargetCount(), 3U);
  EXPECT_EQ(before.name, "Tank-1");
  EXPECT_TRUE(after.pos.x == 200.0 || after.pos.x == 200.1 || after.pos.x == 200.2);
  EXPECT_NEAR(after.velocity.y, 0.0, 1e-9);
}

TEST(DronePhysics, ExecutesQueuedCommandAndUpdatesTelemetry)
{
  DronePhysics physics(Coord{0.0, 0.0}, 10.0, 0.01, 10.0);
  physics.pushCommand(DroneCommand{DroneMode::Accelerating, 1.0, 0.0, 10.0});
  for (int i = 0; i < 20; ++i) {
    physics.stepOnce(0.01);
  }
  const DroneTelemetry telemetry = physics.getTelemetry();
  EXPECT_GT(telemetry.pos.x, 0.0);
  EXPECT_GT(telemetry.speed.x, 0.0);
  EXPECT_GT(telemetry.timeSecSinceStart, 0.0);
}

TEST(MissionProcessor, RunsInOwnThreadAndWritesSimulationJson)
{
  const std::filesystem::path output = std::filesystem::temp_directory_path() / "homework_10_simulation_test.json";
  std::filesystem::remove(output);

  FileConfigLoader config_probe;
  config_probe.load(dataDir().string());
  const MissionConfig cfg = config_probe.getConfig();

  ThreadSafeTargetProvider provider((dataDir() / "targets.json").string(), cfg.simulation.array_time_step, 100.0);
  DronePhysics physics(cfg.drone_pos, cfg.attack_speed, cfg.simulation.physics_time_step, 100.0);
  auto loader = std::make_unique<FileConfigLoader>();
  MissionProcessor mission(std::move(loader), provider, physics, std::make_unique<ConstantSolver>(), output.string());
  mission.init(dataDir().string());

  std::thread provider_thread(&ThreadSafeTargetProvider::run, &provider);
  std::thread physics_thread(&DronePhysics::run, &physics);
  std::thread mission_thread(&MissionProcessor::run, &mission);

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

  ASSERT_TRUE(std::filesystem::exists(output));
  std::ifstream input{output};
  const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("timeSecSinceStart"), std::string::npos);
  EXPECT_NE(text.find("predictedTarget"), std::string::npos);
  std::filesystem::remove(output);
}

}  // namespace homework_10
