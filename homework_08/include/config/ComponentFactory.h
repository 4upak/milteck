#pragma once

// Фабрики для трьох інтерфейсів.
//
// Кожна повертає сирий вказівник на інтерфейс — викликаючий код володіє
// об'єктом і відповідає за delete (як у homework_07, навмисний навчальний
// крок з лекції 14 перед переходом на std::unique_ptr).
//
// enum class описує лише ті реалізації, що зараз існують; додавання нової
// (наприклад TableSolver чи SerialTargetProvider) зведеться до одного case
// у відповідній фабриці.

#include <cstdint>
#include <string>

#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"

namespace homework_08 {

enum class SolverType : std::uint8_t { Analytical };
enum class ProviderType : std::uint8_t { Json };
enum class LoaderType : std::uint8_t { File };

IBallisticSolver* createSolver(SolverType type);
ITargetProvider* createProvider(ProviderType type, const std::string& param);
IConfigLoader* createLoader(LoaderType type);

}  // namespace homework_08
