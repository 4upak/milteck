#include "config/ComponentFactory.h"

#include "config/FileConfigLoader.h"
#include "providers/JsonTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "Types.h"

// Фабрика навмисно віддає сирий вказівник — так задано в лекції 14 (перед
// переходом на std::unique_ptr у наступних заняттях).
// NOLINTBEGIN(cppcoreguidelines-owning-memory)

namespace homework_08 {

IBallisticSolver* createSolver(SolverType type)
{
  switch (type) {
    case SolverType::Analytical:
      return new AnalyticalSolver();
  }
  throw Homework08Error{"createSolver: unsupported solver type"};
}

ITargetProvider* createProvider(ProviderType type, const std::string& param)
{
  switch (type) {
    case ProviderType::Json:
      return new JsonTargetProvider(param);
  }
  throw Homework08Error{"createProvider: unsupported provider type"};
}

IConfigLoader* createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::File:
      return new FileConfigLoader();
  }
  throw Homework08Error{"createLoader: unsupported loader type"};
}

}  // namespace homework_08

// NOLINTEND(cppcoreguidelines-owning-memory)
