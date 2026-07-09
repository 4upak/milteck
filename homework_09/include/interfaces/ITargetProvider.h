#pragma once

// Інтерфейс провайдера цілей.
//
// Контракт: будь-яка реалізація віддає вектор цілей (STL-контейнер замість
// C-масиву + лічильника) і додатково підтримує O(1) пошук за ім'ям —
// типовий приклад std::unordered_map з лекції 16.

#include <string>
#include <vector>

#include "Types.h"

namespace homework_09 {

class ITargetProvider {
public:
  virtual ~ITargetProvider() = default;

  ITargetProvider() = default;
  ITargetProvider(const ITargetProvider&) = delete;
  ITargetProvider& operator=(const ITargetProvider&) = delete;
  ITargetProvider(ITargetProvider&&) = delete;
  ITargetProvider& operator=(ITargetProvider&&) = delete;

  // Повний список цілей у порядку зчитування з джерела. Абоненти можуть
  // ітерувати його через range-based for.
  [[nodiscard]] virtual const std::vector<Target>& targets() const = 0;

  // Пошук цілі за ім'ям; повертає nullptr, якщо такої немає.
  // Реалізація вільна тримати name -> Target в std::unordered_map.
  [[nodiscard]] virtual const Target* findByName(const std::string& name) const = 0;
};

}  // namespace homework_09
