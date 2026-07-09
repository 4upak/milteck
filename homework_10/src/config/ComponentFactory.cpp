#include "config/ComponentFactory.h"

#include <memory>
#include <string>

#include "config/FileConfigLoader.h"
#include "providers/ThreadSafeTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"
#include "Types.h"

namespace homework_10 {

std::unique_ptr<IBallisticSolver> createSolver(SolverType type)
{
  switch (type) {
    case SolverType::Analytical:
      return std::make_unique<AnalyticalSolver>();
    case SolverType::Table:
      return std::make_unique<TableSolver>("homework_10/data/ballistic_table.txt");
  }
  throw Homework10Error{"createSolver: unsupported solver type"};
}

std::unique_ptr<IBallisticSolver> createTableSolver(const std::string& table_path)
{
  return std::make_unique<TableSolver>(table_path);
}

std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& param)
{
  switch (type) {
    case ProviderType::Json:
      return std::make_unique<ThreadSafeTargetProvider>(param, 0.05, 10.0);
  }
  throw Homework10Error{"createProvider: unsupported provider type"};
}

std::unique_ptr<ITargetProvider> createThreadSafeProvider(const std::string& path, double arrayTimeStep, double timeScale)
{
  return std::make_unique<ThreadSafeTargetProvider>(path, arrayTimeStep, timeScale);
}

std::unique_ptr<IConfigLoader> createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::File:
      return std::make_unique<FileConfigLoader>();
  }
  throw Homework10Error{"createLoader: unsupported loader type"};
}

}  // namespace homework_10
