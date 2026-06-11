#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "config/ComponentFactory.h"
#include "config/FileConfigLoader.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "MissionProcessor.h"
#include "providers/JsonTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "Types.h"

// NOLINTBEGIN(cppcoreguidelines-owning-memory)

namespace homework_08 {
namespace {

const std::filesystem::path kDataDir = std::filesystem::path(HOMEWORK_08_DATA_DIR);

AmmoParams reference_vog17_ammo()
{
  return AmmoParams{"VOG-17", 0.35, 0.07, 0.0};
}

TEST(AnalyticalSolver, ProducesKnownDropPoint)
{
  const AnalyticalSolver solver;
  const Coord drone{100.0, 100.0};
  const Coord target{200.0, 200.0};

  const DropPoint drop = solver.solve(drone, target, 100.0, 10.0, reference_vog17_ammo());

  EXPECT_GT(drop.time_of_flight, 0.0);
  EXPECT_GT(drop.horizontal_distance, 0.0);
  EXPECT_NEAR(drop.pos.x, 173.759, 0.01);
  EXPECT_NEAR(drop.pos.y, 173.759, 0.01);
}

TEST(AnalyticalSolver, RejectsZeroAltitude)
{
  const AnalyticalSolver solver;
  EXPECT_THROW((void)solver.solve(Coord{0, 0}, Coord{100, 0}, 0.0, 10.0, reference_vog17_ammo()), Homework08Error);
}

TEST(AnalyticalSolver, RejectsCoincidentDroneAndTarget)
{
  const AnalyticalSolver solver;
  EXPECT_THROW((void)solver.solve(Coord{50, 50}, Coord{50, 50}, 100.0, 10.0, reference_vog17_ammo()), Homework08Error);
}

TEST(JsonTargetProvider, ExposesTargetsAsVector)
{
  const JsonTargetProvider provider{(kDataDir / "targets.json").string()};

  const std::vector<Target>& items = provider.targets();
  ASSERT_EQ(items.size(), 5U);
  EXPECT_EQ(items.front().name, "Tank-1");
  EXPECT_DOUBLE_EQ(items.front().pos.x, 200.0);
  EXPECT_DOUBLE_EQ(items.front().pos.y, 150.0);
  EXPECT_DOUBLE_EQ(items.front().velocity.x, 2.0);
}

TEST(JsonTargetProvider, RangeBasedForVisitsAllTargets)
{
  const JsonTargetProvider provider{(kDataDir / "targets.json").string()};

  std::size_t count = 0;
  for (const Target& t : provider.targets()) {
    EXPECT_FALSE(t.name.empty());
    ++count;
  }
  EXPECT_EQ(count, provider.targets().size());
}

TEST(JsonTargetProvider, FindByNameUsesHashTable)
{
  const JsonTargetProvider provider{(kDataDir / "targets.json").string()};

  const Target* found = provider.findByName("Bunker-4");
  ASSERT_NE(found, nullptr);
  EXPECT_DOUBLE_EQ(found->pos.x, 500.0);
  EXPECT_DOUBLE_EQ(found->pos.y, 500.0);

  EXPECT_EQ(provider.findByName("does-not-exist"), nullptr);
}

TEST(JsonTargetProvider, ThrowsOnMissingFile)
{
  EXPECT_THROW(JsonTargetProvider{"/nonexistent/targets.json"}, Homework08Error);
}

TEST(FileConfigLoader, LoadsConfigAndResolvesAmmo)
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
  EXPECT_DOUBLE_EQ(ammo.drag, 0.07);
}

TEST(FileConfigLoader, AmmoTableContainsAllEntries)
{
  FileConfigLoader loader;
  loader.load(kDataDir.string());

  const std::unordered_map<std::string, AmmoParams>& table = loader.ammoTable();
  EXPECT_GE(table.size(), 5U);
  EXPECT_TRUE(table.contains("VOG-17"));
  EXPECT_TRUE(table.contains("M67"));
  EXPECT_TRUE(table.contains("RKG-3"));
  EXPECT_TRUE(table.contains("GLIDING-VOG"));
  EXPECT_TRUE(table.contains("GLIDING-RKG"));
  EXPECT_FALSE(table.contains("NOT-AN-AMMO"));
}

TEST(FileConfigLoader, GettersFailBeforeLoad)
{
  FileConfigLoader loader;
  EXPECT_THROW((void)loader.getConfig(), Homework08Error);
  EXPECT_THROW((void)loader.getAmmoParams(), Homework08Error);
  EXPECT_THROW((void)loader.ammoTable(), Homework08Error);
}

TEST(FileConfigLoader, ThrowsOnUnknownAmmo)
{
  const std::filesystem::path tmp = std::filesystem::temp_directory_path() / "hw08_unknown";
  std::filesystem::create_directories(tmp);
  std::filesystem::copy_file(kDataDir / "invalid_ammo_config.json", tmp / "config.json", std::filesystem::copy_options::overwrite_existing);
  std::filesystem::copy_file(kDataDir / "ammo.json", tmp / "ammo.json", std::filesystem::copy_options::overwrite_existing);

  FileConfigLoader loader;
  EXPECT_THROW(loader.load(tmp.string()), Homework08Error);
}

TEST(Factory, CreatesAndDestroysAllThree)
{
  IConfigLoader* loader = createLoader(LoaderType::File);
  ITargetProvider* provider = createProvider(ProviderType::Json, (kDataDir / "targets.json").string());
  IBallisticSolver* solver = createSolver(SolverType::Analytical);

  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(solver, nullptr);
  EXPECT_EQ(provider->targets().size(), 5U);

  delete solver;
  delete provider;
  delete loader;
}

TEST(MissionProcessor, IteratesAllTargetsThenStops)
{
  IConfigLoader* loader = createLoader(LoaderType::File);
  ITargetProvider* provider = createProvider(ProviderType::Json, (kDataDir / "targets.json").string());
  IBallisticSolver* solver = createSolver(SolverType::Analytical);

  MissionProcessor mission(loader, provider, solver);
  mission.init(kDataDir.string());

  std::size_t processed = 0;
  while (mission.hasNext()) {
    const DropPoint drop = mission.step();
    EXPECT_GT(drop.time_of_flight, 0.0);
    EXPECT_GT(drop.horizontal_distance, 0.0);
    ++processed;
  }
  EXPECT_EQ(processed, provider->targets().size());
  EXPECT_FALSE(mission.hasNext());
  EXPECT_THROW((void)mission.step(), Homework08Error);

  delete solver;
  delete provider;
  delete loader;
}

TEST(MissionProcessor, ResetRestartsIteration)
{
  IConfigLoader* loader = createLoader(LoaderType::File);
  ITargetProvider* provider = createProvider(ProviderType::Json, (kDataDir / "targets.json").string());
  IBallisticSolver* solver = createSolver(SolverType::Analytical);

  MissionProcessor mission(loader, provider, solver);
  mission.init(kDataDir.string());
  (void)mission.step();
  (void)mission.step();
  EXPECT_EQ(mission.currentIndex(), 2U);

  mission.reset();
  EXPECT_EQ(mission.currentIndex(), 0U);
  EXPECT_TRUE(mission.hasNext());

  delete solver;
  delete provider;
  delete loader;
}

// Тестовий solver-замінник для перевірки патерну Стратегія.
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

TEST(MissionProcessor, ChangeSolverSwapsStrategyOnTheFly)
{
  IConfigLoader* loader = createLoader(LoaderType::File);
  ITargetProvider* provider = createProvider(ProviderType::Json, (kDataDir / "targets.json").string());
  IBallisticSolver* analytic = createSolver(SolverType::Analytical);
  auto counting = std::make_unique<CountingSolver>();

  MissionProcessor mission(loader, provider, analytic);
  mission.init(kDataDir.string());

  (void)mission.step();
  mission.changeSolver(counting.get());

  const DropPoint drop = mission.step();
  EXPECT_DOUBLE_EQ(drop.pos.x, 42.0);
  EXPECT_DOUBLE_EQ(drop.pos.y, 24.0);
  EXPECT_EQ(counting->calls, 1);

  delete analytic;
  delete provider;
  delete loader;
}

TEST(MissionProcessor, RejectsNullDependencies)
{
  IConfigLoader* loader = createLoader(LoaderType::File);
  ITargetProvider* provider = createProvider(ProviderType::Json, (kDataDir / "targets.json").string());
  IBallisticSolver* solver = createSolver(SolverType::Analytical);

  EXPECT_THROW(MissionProcessor(nullptr, provider, solver), Homework08Error);
  EXPECT_THROW(MissionProcessor(loader, nullptr, solver), Homework08Error);
  EXPECT_THROW(MissionProcessor(loader, provider, nullptr), Homework08Error);

  delete solver;
  delete provider;
  delete loader;
}

}  // namespace
}  // namespace homework_08

// NOLINTEND(cppcoreguidelines-owning-memory)
