#include "factory.hpp"

#include "analytical_solver.hpp"
#include "file_config_loader.hpp"
#include "json_target_provider.hpp"
#include "types.hpp"

// Фабрика навмисно віддає сирий вказівник — так задано в умові ДЗ і так
// показано в лекції 14. clang-tidy на cppcoreguidelines-owning-memory тут
// глушимо локально, бо саме цей патерн ми і вчимо перед переходом на
// std::unique_ptr у наступних заняттях.
// NOLINTBEGIN(cppcoreguidelines-owning-memory)

namespace homework_07 {

IBallisticSolver* createSolver(SolverType type)
{
  switch (type) {
    case SolverType::Analytical:
      return new AnalyticalSolver();
  }
  throw Homework07Error{"createSolver: unsupported solver type"};
}

ITargetProvider* createProvider(ProviderType type, const std::string& param)
{
  switch (type) {
    case ProviderType::Json:
      return new JsonTargetProvider(param);
  }
  throw Homework07Error{"createProvider: unsupported provider type"};
}

IConfigLoader* createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::File:
      return new FileConfigLoader();
  }
  throw Homework07Error{"createLoader: unsupported loader type"};
}

}  // namespace homework_07

// NOLINTEND(cppcoreguidelines-owning-memory)
