#pragma once

// Спільні доменні типи для homework_09.
//
// Тримає тільки маленькі POD-структури (дані без поведінки) і доменний тип
// помилки. Усе, що стосується I/O, балістики чи стратегії, живе у відповідних
// реалізаціях у підпапках include/{interfaces,solvers,providers,config}.

#include <stdexcept>
#include <string>

namespace homework_09 {

// 2D-координата на горизонтальній площині.
struct Coord {
  double x = 0.0;
  double y = 0.0;
};

// Опис однієї цілі: де вона зараз і куди рухається.
struct Target {
  Coord pos{};
  Coord velocity{};
  std::string name;
};

// Фізичні параметри одного боєприпасу.
struct AmmoParams {
  std::string name;
  double mass = 0.0;
  double drag = 0.0;
  double lift = 0.0;
};

// Конфігурація місії: початковий стан дрона і вибір боєприпасу.
struct MissionConfig {
  Coord drone_pos{};
  double altitude = 0.0;
  double attack_speed = 0.0;
  std::string ammo_name;
};

// Результат балістичного обчислення для однієї цілі.
struct DropPoint {
  Coord pos{};                       // Куди скидати боєприпас (у горизонтальній площині)
  double time_of_flight = 0.0;       // Скільки летить боєприпас до удару
  double horizontal_distance = 0.0;  // Горизонтальна дальність боєприпасу
};

// Доменна помилка: некоректний конфіг, невідомий боєприпас, неможлива
// геометрія тощо. CLI ловить її окремо від інших винятків.
class Homework09Error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

}  // namespace homework_09
