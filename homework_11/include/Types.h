#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include "protocol/drone_link.h"

namespace homework_11 {

struct RuntimeConfig {
  std::string uart_device = "/tmp/ttyA";
  std::string gpio_chip = "gpiochip0";
  unsigned start_line = 24;
  unsigned drop_line = 23;
  unsigned control_period_ms = 20;
  unsigned drop_hold_us = 80000;
};

struct Coord {
  double x = 0.0;
  double y = 0.0;
};

inline Coord operator+(Coord lhs, Coord rhs) { return Coord{lhs.x + rhs.x, lhs.y + rhs.y}; }
inline Coord operator-(Coord lhs, Coord rhs) { return Coord{lhs.x - rhs.x, lhs.y - rhs.y}; }
inline Coord operator*(Coord lhs, double scale) { return Coord{lhs.x * scale, lhs.y * scale}; }

struct GuidanceCommand {
  float accel = 0.0F;
  float turnRate = 0.0F;
};

struct TargetTrack {
  bool seen = false;
  bool done = false;
  std::uint8_t id = 0;
  Coord pos{};
  Coord velocity{};
  std::uint32_t t_ms = 0;
};

struct DropSolution {
  bool valid = false;
  Coord target_now{};
  Coord predicted_target{};
  Coord drop_point{};
  double distance_to_drop = 0.0;
  double distance_to_target = 0.0;
  double lateral_error = 0.0;
  double heading_error = 0.0;
  double time_of_flight = 0.0;
  double horizontal_distance = 0.0;
};

struct MissionSnapshot {
  std::optional<dlink::Telemetry> telemetry;
  std::optional<dlink::AmmoCfg> ammo;
  std::optional<dlink::DroneCfg> config;
  std::optional<dlink::Result> result;
  std::array<TargetTrack, 32> targets{};
};

class Homework11Error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

}  // namespace homework_11
