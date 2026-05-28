#pragma once

#include <string>

#include "i_config_loader.hpp"
#include "types.hpp"

namespace homework_07 {

// Читає config.json і ammo.json із заданої теки.
// configSource — шлях до теки, де лежать обидва файли.
// За назвою боєприпасу з config.json підбирає рядок з ammo.json і кешує його.
class FileConfigLoader : public IConfigLoader {
public:
  void load(const std::string& source) override;
  [[nodiscard]] const MissionConfig& getConfig() const override;
  [[nodiscard]] const AmmoParams& getAmmoParams() const override;

private:
  MissionConfig config_{};
  AmmoParams ammo_{};
  bool loaded_ = false;
};

}  // namespace homework_07
