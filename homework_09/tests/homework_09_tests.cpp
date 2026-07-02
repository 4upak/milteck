#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "config/ComponentFactory.h"
#include "config/FileConfigLoader.h"
#include "drone/DroneState.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "MissionProcessor.h"
#include "providers/JsonTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"
#include "Types.h"

namespace homework_09 {
namespace {

const std::filesystem::path kDataDir = std::filesystem::path(HOMEWORK_09_DATA_DIR);

AmmoParams reference_vog17_ammo()
{
  return AmmoParams{"VOG-17", 0.35, 0.004, 0.0};
}

BallisticTable make_linear_test_table()
{
  BallisticTable table;
  table.axisZ0 = {0.0, 10.0};
  table.axisV0 = {0.0, 20.0};
  table.axisM = {0.0, 2.0};
  table.axisD = {0.0, 4.0};
  table.axisL = {0.0, 8.0};
  table.data.reserve(32);

  for (double z : table.axisZ0) {
    for (double v : table.axisV0) {
      for (double m : table.axisM) {
        for (double d : table.axisD) {
          for (double l : table.axisL) {
            const double sum = z + v + m + d + l;
            table.data.push_back(BallisticTableResult{1.0 + sum, 2.0 + 2.0 * sum});
          }
        }
      }
    }
  }
  return table;
}

TEST(AnalyticalSolver, ProducesPositiveDropPoint)
{
  const AnalyticalSolver solver;
  const Coord drone{100.0, 100.0};
  const Coord target{200.0, 200.0};

  const DropPoint drop = solver.solve(drone, target, 100.0, 10.0, reference_vog17_ammo());

  EXPECT_GT(drop.time_of_flight, 0.0);
  EXPECT_GT(drop.horizontal_distance, 0.0);
  EXPECT_TRUE(std::isfinite(drop.pos.x));
  EXPECT_TRUE(std::isfinite(drop.pos.y));
}

TEST(JsonTargetProvider, ExposesTargetsAsVectorAndHashLookup)
{
  const JsonTargetProvider provider{(kDataDir / "targets.json").string()};

  const std::vector<Target>& items = provider.targets();
  ASSERT_EQ(items.size(), 5U);
  EXPECT_EQ(items.front().name, "Tank-1");

  const Target* found = provider.findByName("Bunker-4");
  ASSERT_NE(found, nullptr);
  EXPECT_DOUBLE_EQ(found->pos.x, 500.0);
  EXPECT_EQ(provider.findByName("does-not-exist"), nullptr);
}

TEST(FileConfigLoader, LoadsUpdatedAmmoParameters)
{
  FileConfigLoader loader;
  loader.load(kDataDir.string());

  const MissionConfig& cfg = loader.getConfig();
  EXPECT_DOUBLE_EQ(cfg.altitude, 120.0);
  EXPECT_DOUBLE_EQ(cfg.attack_speed, 30.0);
  EXPECT_EQ(cfg.ammo_name, "VOG-17");

  const AmmoParams& ammo = loader.getAmmoParams();
  EXPECT_EQ(ammo.name, "VOG-17");
  EXPECT_DOUBLE_EQ(ammo.mass, 0.35);
  EXPECT_DOUBLE_EQ(ammo.drag, 0.004);
  EXPECT_DOUBLE_EQ(ammo.lift, 0.0);

  const std::unordered_map<std::string, AmmoParams>& table = loader.ammoTable();
  EXPECT_TRUE(table.contains("GLIDING-RKG"));
  EXPECT_DOUBLE_EQ(table.at("GLIDING-RKG").lift, 0.005);
}

TEST(FileConfigLoader, GettersFailBeforeLoadAndUnknownAmmoThrows)
{
  FileConfigLoader loader;
  EXPECT_THROW((void)loader.getConfig(), Homework09Error);
  EXPECT_THROW((void)loader.getAmmoParams(), Homework09Error);
  EXPECT_THROW((void)loader.ammoTable(), Homework09Error);

  const std::filesystem::path tmp = std::filesystem::temp_directory_path() / "hw09_unknown";
  std::filesystem::create_directories(tmp);
  std::filesystem::copy_file(kDataDir / "invalid_ammo_config.json", tmp / "config.json", std::filesystem::copy_options::overwrite_existing);
  std::filesystem::copy_file(kDataDir / "ammo.json", tmp / "ammo.json", std::filesystem::copy_options::overwrite_existing);

  EXPECT_THROW(loader.load(tmp.string()), Homework09Error);
}

TEST(Factory, ReturnsUniquePointers)
{
  std::unique_ptr<IConfigLoader> loader = createLoader(LoaderType::File);
  std::unique_ptr<ITargetProvider> provider = createProvider(ProviderType::Json, (kDataDir / "targets.json").string());
  std::unique_ptr<IBallisticSolver> analytical = createSolver(SolverType::Analytical);
  std::unique_ptr<IBallisticSolver> table = createTableSolver((kDataDir / "ballistic_table.txt").string());

  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(analytical, nullptr);
  ASSERT_NE(table, nullptr);
  EXPECT_EQ(provider->targets().size(), 5U);
}

TEST(BallisticTable, InterpolatesFiveDimensionsAndClamps)
{
  const BallisticTable table = make_linear_test_table();

  const BallisticTableResult mid = table.lookup(5.0, 10.0, 1.0, 2.0, 4.0);
  EXPECT_DOUBLE_EQ(mid.time_of_flight, 23.0);
  EXPECT_DOUBLE_EQ(mid.horizontal_distance, 46.0);

  const BallisticTableResult clamped = table.lookup(-100.0, 100.0, 1.0, 2.0, 4.0);
  EXPECT_DOUBLE_EQ(clamped.time_of_flight, 28.0);
  EXPECT_DOUBLE_EQ(clamped.horizontal_distance, 56.0);
}

TEST(TableSolver, UsesTableResultForDropPoint)
{
  TableSolver solver{make_linear_test_table()};

  const DropPoint drop = solver.solve(Coord{0.0, 0.0}, Coord{100.0, 0.0}, 5.0, 10.0, AmmoParams{"X", 1.0, 2.0, 4.0});

  EXPECT_DOUBLE_EQ(drop.time_of_flight, 23.0);
  EXPECT_DOUBLE_EQ(drop.horizontal_distance, 46.0);
  EXPECT_DOUBLE_EQ(drop.pos.x, 54.0);
  EXPECT_DOUBLE_EQ(drop.pos.y, 0.0);
}

TEST(MissionProcessor, OwnsComponentsAndRunsTimeStepSimulation)
{
  MissionProcessor mission(
    createLoader(LoaderType::File),
    createProvider(ProviderType::Json, (kDataDir / "targets.json").string()),
    createSolver(SolverType::Analytical));
  mission.init(kDataDir.string());

  const Coord start = mission.dronePosition();
  std::size_t steps = 0;
  bool saw_state_machine_work = false;
  while (mission.hasNext()) {
    const std::size_t target_before = mission.currentIndex();
    const DropPoint drop = mission.step();
    EXPECT_GT(drop.time_of_flight, 0.0);
    EXPECT_GT(drop.horizontal_distance, 0.0);
    EXPECT_GE(mission.currentIndex(), target_before);
    saw_state_machine_work = saw_state_machine_work || std::string{mission.stateName()} != "Stopped";
    ++steps;
  }

  EXPECT_GT(steps, mission.targets().size());
  EXPECT_GT(mission.currentTime(), 0.0);
  EXPECT_TRUE(saw_state_machine_work);
  EXPECT_NE(mission.dronePosition().x, start.x);
  EXPECT_NE(mission.dronePosition().y, start.y);
  EXPECT_THROW((void)mission.step(), Homework09Error);
}

class CountingSolver : public IBallisticSolver {
public:
  mutable int calls = 0;
  DropPoint solve(
    Coord /*drone_pos*/, Coord /*target_pos*/, double /*altitude*/, double /*attack_speed*/, const AmmoParams& /*ammo*/) const override
  {
    ++calls;
    return DropPoint{Coord{42.0, 24.0}, 1.0, 2.0};
  }
};

TEST(MissionProcessor, ChangeSolverTakesUniquePtrOwnership)
{
  MissionProcessor mission(
    createLoader(LoaderType::File),
    createProvider(ProviderType::Json, (kDataDir / "targets.json").string()),
    createSolver(SolverType::Analytical));
  mission.init(kDataDir.string());

  (void)mission.step();
  auto counting = std::make_unique<CountingSolver>();
  const CountingSolver* observer = counting.get();
  mission.changeSolver(std::move(counting));
  EXPECT_EQ(counting, nullptr);

  const DropPoint drop = mission.step();
  EXPECT_DOUBLE_EQ(drop.pos.x, 42.0);
  EXPECT_DOUBLE_EQ(drop.pos.y, 24.0);
  EXPECT_EQ(observer->calls, 1);
}

TEST(DroneState, StoppedTransitionsToTurningWhenDirectionDiffers)
{
  DroneContext ctx;
  ctx.direction = 0.0;
  ctx.desired_direction = 1.0;
  ctx.cfg.angular_speed = 2.0;
  ctx.cfg.turn_threshold = 0.01;

  StateStopped stopped;
  std::unique_ptr<IDroneState> next = stopped.execute(ctx);

  ASSERT_NE(next, nullptr);
  EXPECT_STREQ(next->name(), "Turning");
  EXPECT_NEAR(ctx.target_direction, 1.0, 1e-9);
  EXPECT_NEAR(ctx.turn_remaining, 0.5, 1e-9);
}

TEST(DroneState, AcceleratingMovingAndDeceleratingFlow)
{
  DroneContext ctx;
  ctx.cfg.attack_speed = 4.0;
  ctx.cfg.acceleration = 2.0;
  ctx.cfg.sim_time_step = 1.0;
  ctx.cfg.turn_threshold = 0.01;
  ctx.direction = 0.0;
  ctx.desired_direction = 0.0;

  std::unique_ptr<IDroneState> state = std::make_unique<StateAccelerating>();
  std::unique_ptr<IDroneState> next = state->execute(ctx);
  EXPECT_EQ(next, nullptr);
  EXPECT_DOUBLE_EQ(ctx.speed, 2.0);

  next = state->execute(ctx);
  ASSERT_NE(next, nullptr);
  EXPECT_STREQ(next->name(), "Moving");
  state = std::move(next);

  ctx.desired_direction = 1.0;
  next = state->execute(ctx);
  ASSERT_NE(next, nullptr);
  EXPECT_STREQ(next->name(), "Decelerating");
}

TEST(DroneState, TurningFinishesIntoAccelerating)
{
  DroneContext ctx;
  ctx.direction = 0.0;
  ctx.target_direction = 0.1;
  ctx.cfg.angular_speed = 1.0;
  ctx.cfg.sim_time_step = 1.0;
  ctx.cfg.turn_threshold = 0.01;

  StateTurning turning;
  std::unique_ptr<IDroneState> next = turning.execute(ctx);

  ASSERT_NE(next, nullptr);
  EXPECT_STREQ(next->name(), "Accelerating");
  EXPECT_NEAR(ctx.direction, 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(ctx.turn_remaining, 0.0);
}

TEST(MissionProcessor, RejectsNullDependencies)
{
  EXPECT_THROW(MissionProcessor(nullptr, createProvider(ProviderType::Json, (kDataDir / "targets.json").string()), createSolver(SolverType::Analytical)),
               Homework09Error);
  EXPECT_THROW(MissionProcessor(createLoader(LoaderType::File), nullptr, createSolver(SolverType::Analytical)), Homework09Error);
  EXPECT_THROW(MissionProcessor(createLoader(LoaderType::File), createProvider(ProviderType::Json, (kDataDir / "targets.json").string()), nullptr),
               Homework09Error);
}

}  // namespace
}  // namespace homework_09
