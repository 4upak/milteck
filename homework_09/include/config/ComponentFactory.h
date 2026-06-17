#pragma once

// Фабрики для homework_09.
//
// У ДЗ9 фабрики більше не повертають owning raw pointer. Власність явно
// передається через std::unique_ptr, а викликаючий код рухає об'єкти через
// std::move().

#include <cstdint>
#include <memory>
#include <string>

#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"

namespace homework_09 {

enum class SolverType : std::uint8_t { Analytical, Table };
enum class ProviderType : std::uint8_t { Json };
enum class LoaderType : std::uint8_t { File };

std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
std::unique_ptr<IBallisticSolver> createTableSolver(const std::string& table_path);
std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& param);
std::unique_ptr<IConfigLoader> createLoader(LoaderType type);

}  // namespace homework_09
