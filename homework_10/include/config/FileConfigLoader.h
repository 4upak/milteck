#pragma once

// FileConfigLoader — реалізація IConfigLoader, що читає config.json і
// ammo.json із заданої теки.
//
// Замість лінійного пошуку по масиву боєприпасів (як у homework_07)
// усі боєприпаси завантажуються в std::unordered_map<std::string, AmmoParams>.
// Це і є канонічна заміна if/else-ланцюжків / лінійного перебору на
// hash-таблицю з лекції 16.

#include <string>
#include <unordered_map>

#include "interfaces/IConfigLoader.h"
#include "Types.h"

namespace homework_10 {

class FileConfigLoader : public IConfigLoader {
public:
  void load(const std::string& source) override;
  [[nodiscard]] const MissionConfig& getConfig() const override;
  [[nodiscard]] const AmmoParams& getAmmoParams() const override;

  // Допоміжний геттер для тестів і CLI: повна таблиця завантажених
  // боєприпасів. Дає змогу подивитися, які типи доступні без додаткового
  // читання файла.
  [[nodiscard]] const std::unordered_map<std::string, AmmoParams>& ammoTable() const;

private:
  MissionConfig config_{};
  AmmoParams selected_ammo_{};
  std::unordered_map<std::string, AmmoParams> ammo_table_;
  bool loaded_ = false;
};

}  // namespace homework_10
