#pragma once

// JsonTargetProvider — реалізація ITargetProvider, що читає список цілей
// з JSON-файла такого вигляду:
//
//   {
//     "targets": [
//       {"name": "Tank-1", "pos": {"x": 90, "y": 200},
//        "velocity": {"x": 1.5, "y": -0.5}}
//     ]
//   }
//
// Внутрішнє сховище — std::vector<Target>. Додатково тримається
// std::unordered_map<std::string, std::size_t> для O(1) пошуку за ім'ям —
// типовий приклад з лекції 16 (хеш-таблиця для lookup за ID/назвою).

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "interfaces/ITargetProvider.h"
#include "Types.h"

namespace homework_09 {

class JsonTargetProvider : public ITargetProvider {
public:
  explicit JsonTargetProvider(const std::string& path);

  [[nodiscard]] const std::vector<Target>& targets() const override;
  [[nodiscard]] const Target* findByName(const std::string& name) const override;

private:
  std::vector<Target> targets_;
  std::unordered_map<std::string, std::size_t> name_to_index_;
};

}  // namespace homework_09
