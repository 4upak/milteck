#pragma once

#include <string>
#include <vector>

#include "i_target_provider.hpp"
#include "types.hpp"

namespace homework_07 {

// Зчитує цілі з JSON-файла такого вигляду:
//   {
//     "targets": [
//       {"name": "Tank-1", "pos": {"x": 90, "y": 200},
//        "velocity": {"x": 1.5, "y": -0.5}}
//     ]
//   }
class JsonTargetProvider : public ITargetProvider {
public:
  explicit JsonTargetProvider(const std::string& path);

  [[nodiscard]] int getTargetCount() const override;
  [[nodiscard]] Target getTarget(int index) const override;

private:
  std::vector<Target> targets_;
};

}  // namespace homework_07
