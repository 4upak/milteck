#pragma once

// Інтерфейс провайдера цілей.
//
// Контракт: будь-яка реалізація вміє сказати, скільки цілей є, і повернути
// дані конкретної цілі за індексом. JsonTargetProvider читає файл,
// TestTargetProvider у тестах віддає захардкоджені дані — викликаючий код
// нічого про це не знає.

#include "types.hpp"

namespace homework_07 {

class ITargetProvider {
public:
  virtual ~ITargetProvider() = default;

  ITargetProvider() = default;
  ITargetProvider(const ITargetProvider&) = delete;
  ITargetProvider& operator=(const ITargetProvider&) = delete;
  ITargetProvider(ITargetProvider&&) = delete;
  ITargetProvider& operator=(ITargetProvider&&) = delete;

  [[nodiscard]] virtual int getTargetCount() const = 0;
  [[nodiscard]] virtual Target getTarget(int index) const = 0;
};

}  // namespace homework_07
