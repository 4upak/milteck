#pragma once

// Інтерфейс завантажувача конфігурації.
//
// Контракт: load(source) читає конфіг із джерела (файл, тестовий хардкод,
// мережа — для викликаючого коду це непрозоро). Після успішного load()
// getConfig() і getAmmoParams() повертають дані для подальшого використання.
// До load() обидва геттери кидають Homework07Error.

#include <string>

#include "types.hpp"

namespace homework_07 {

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

}  // namespace homework_07
