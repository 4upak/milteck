#pragma once

// Фабрики для трьох інтерфейсів.
//
// Кожна повертає сирий вказівник на інтерфейс — викликаючий код володіє
// об'єктом і відповідає за delete. Тип enum class описує лише ті реалізації,
// що зараз існують; додавання нової (наприклад TableSolver чи
// SerialTargetProvider) зведеться до одного case у відповідній фабриці.

#include <cstdint>
#include <string>

#include "i_ballistic_solver.hpp"
#include "i_config_loader.hpp"
#include "i_target_provider.hpp"

namespace homework_07 {

enum class SolverType : std::uint8_t { Analytical };
enum class ProviderType : std::uint8_t { Json };
enum class LoaderType : std::uint8_t { File };

IBallisticSolver* createSolver(SolverType type);
ITargetProvider* createProvider(ProviderType type, const std::string& param);
IConfigLoader* createLoader(LoaderType type);

}  // namespace homework_07
