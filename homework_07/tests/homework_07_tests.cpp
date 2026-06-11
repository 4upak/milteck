#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "analytical_solver.hpp"
#include "factory.hpp"
#include "file_config_loader.hpp"
#include "i_ballistic_solver.hpp"
#include "i_config_loader.hpp"
#include "i_target_provider.hpp"
#include "json_target_provider.hpp"
#include "mission_processor.hpp"
#include "types.hpp"

// Тести навмисно використовують raw pointer + delete: саме цей патерн фабрики
// з лекції 14 ми і перевіряємо. Глушимо відповідний clang-tidy check на весь
// файл — інакше кожен тестовий блок створив би шум на 3-4 рядки.
// NOLINTBEGIN(cppcoreguidelines-owning-memory)

namespace homework_07 {
namespace {

const std::filesystem::path kDataDir = std::filesystem::path(HOMEWORK_07_DATA_DIR);

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
  EXPECT_THROW((void)solver.solve(Coord{0, 0}, Coord{100, 0}, 0.0, 10.0, reference_vog17_ammo()), Homework07Error);
}

TEST(AnalyticalSolver, RejectsCoincidentDroneAndTarget)
{
  const AnalyticalSolver solver;
  EXPECT_THROW((void)solver.solve(Coord{50, 50}, Coord{50, 50}, 100.0, 10.0, reference_vog17_ammo()), Homework07Error);
}

TEST(JsonTargetProvider, LoadsAllSampleTargets)
{
  const JsonTargetProvider provider{(kDataDir / "targets.json").string()};

  ASSERT_EQ(provider.getTargetCount(), 5);
  const Target first = provider.getTarget(0);
  EXPECT_EQ(first.name, "Tank-1");
  EXPECT_DOUBLE_EQ(first.pos.x, 200.0);
  EXPECT_DOUBLE_EQ(first.pos.y, 150.0);
  EXPECT_DOUBLE_EQ(first.velocity.x, 2.0);
}

TEST(JsonTargetProvider, ThrowsOnMissingFile)
{
  EXPECT_THROW(JsonTargetProvider{"/nonexistent/targets.json"}, Homework07Error);
}

TEST(JsonTargetProvider, RejectsOutOfRangeIndex)
{
  const JsonTargetProvider provider{(kDataDir / "targets.json").string()};
  EXPECT_THROW((void)provider.getTarget(-1), Homework07Error);
  EXPECT_THROW((void)provider.getTarget(provider.getTargetCount()), Homework07Error);
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

TEST(FileConfigLoader, GettersFailBeforeLoad)
{
  FileConfigLoader loader;
  EXPECT_THROW((void)loader.getConfig(), Homework07Error);
  EXPECT_THROW((void)loader.getAmmoParams(), Homework07Error);
}

TEST(FileConfigLoader, ThrowsOnUnknownAmmo)
{
  const std::filesystem::path tmp = std::filesystem::temp_directory_path() / "hw07_unknown";
  std::filesystem::create_directories(tmp);
  std::filesystem::copy_file(kDataDir / "invalid_ammo_config.json", tmp / "config.json", std::filesystem::copy_options::overwrite_existing);
  std::filesystem::copy_file(kDataDir / "ammo.json", tmp / "ammo.json", std::filesystem::copy_options::overwrite_existing);

  FileConfigLoader loader;
  EXPECT_THROW(loader.load(tmp.string()), Homework07Error);
}

TEST(Factory, CreatesAndDestroysAllThree)
{
  IConfigLoader* loader = createLoader(LoaderType::File);
  ITargetProvider* provider = createProvider(ProviderType::Json, (kDataDir / "targets.json").string());
  IBallisticSolver* solver = createSolver(SolverType::Analytical);

  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(solver, nullptr);
  EXPECT_EQ(provider->getTargetCount(), 5);

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

  int processed = 0;
  while (mission.hasNext()) {
    const DropPoint drop = mission.step();
    EXPECT_GT(drop.time_of_flight, 0.0);
    EXPECT_GT(drop.horizontal_distance, 0.0);
    ++processed;
  }
  EXPECT_EQ(processed, provider->getTargetCount());
  EXPECT_FALSE(mission.hasNext());
  EXPECT_THROW((void)mission.step(), Homework07Error);

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
  EXPECT_EQ(mission.currentIndex(), 2);

  mission.reset();
  EXPECT_EQ(mission.currentIndex(), 0);
  EXPECT_TRUE(mission.hasNext());

  delete solver;
  delete provider;
  delete loader;
}

// Тестовий solver-проб'єник: повертає фіксований DropPoint і рахує виклики.
// Існування цього класу — мала демонстрація сили інтерфейсу: ми можемо
// підмінити реальний AnalyticalSolver на mock у тестах без жодної зміни
// MissionProcessor.
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

  EXPECT_THROW(MissionProcessor(nullptr, provider, solver), Homework07Error);
  EXPECT_THROW(MissionProcessor(loader, nullptr, solver), Homework07Error);
  EXPECT_THROW(MissionProcessor(loader, provider, nullptr), Homework07Error);

  delete solver;
  delete provider;
  delete loader;
}

}  // namespace
}  // namespace homework_07

// NOLINTEND(cppcoreguidelines-owning-memory)
