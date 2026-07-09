#pragma once

// Інтерфейс завантажувача конфігурації.
//
// Контракт: load(source) читає конфіг із джерела (файл, тестовий хардкод
// тощо). Після успішного load() getConfig() і getAmmoParams() повертають
// дані. До load() обидва геттери кидають Homework09Error.

#include <string>

#include "Types.h"

namespace homework_09 {

class IConfigLoader {
public:
  virtual ~IConfigLoader() = default;

  IConfigLoader() = default;
  IConfigLoader(const IConfigLoader&) = delete;
  IConfigLoader& operator=(const IConfigLoader&) = delete;
  IConfigLoader(IConfigLoader&&) = delete;
  IConfigLoader& operator=(IConfigLoader&&) = delete;

  virtual void load(const std::string& source) = 0;
  [[nodiscard]] virtual const MissionConfig& getConfig() const = 0;
  [[nodiscard]] virtual const AmmoParams& getAmmoParams() const = 0;
};

}  // namespace homework_09
