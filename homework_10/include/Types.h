#pragma once

#include <stdexcept>
#include <string>

namespace homework_10 {

struct Coord {
  double x = 0.0;
  double y = 0.0;
};

inline Coord operator+(Coord lhs, Coord rhs) { return Coord{lhs.x + rhs.x, lhs.y + rhs.y}; }
inline Coord operator-(Coord lhs, Coord rhs) { return Coord{lhs.x - rhs.x, lhs.y - rhs.y}; }
inline Coord operator*(Coord lhs, double scale) { return Coord{lhs.x * scale, lhs.y * scale}; }

// ДЗ10: Target назовні містить тільки поточний snapshot: позицію і швидкість.
// Траєкторії залишаються приватними даними ThreadSafeTargetProvider.
struct Target {
  Coord pos{};
  Coord velocity{};
  std::string name;
};

struct AmmoParams {
  std::string name;
  double mass = 0.0;
  double drag = 0.0;
  double lift = 0.0;
};

struct SimulationConfig {
  double sim_time_step = 0.05;
  double array_time_step = 0.05;
  double target_time_step = 0.05;
  double physics_time_step = 0.01;
  double time_scale = 10.0;
  double max_mission_time = 5.0;
};

struct MissionConfig {
  Coord drone_pos{};
  double altitude = 0.0;
  double attack_speed = 0.0;
  std::string ammo_name;
  SimulationConfig simulation{};
};

struct DropPoint {
  Coord pos{};
  double time_of_flight = 0.0;
  double horizontal_distance = 0.0;
};

enum class DroneMode { Stopped, Accelerating, Decelerating, Turning, Moving };

struct DroneCommand {
  DroneMode state = DroneMode::Stopped;
  double angleSpeed = 0.0;
  double desiredDirection = 0.0;
  double targetSpeed = 0.0;
};

struct DroneTelemetry {
  Coord pos{};
  Coord speed{};
  double direction = 0.0;
  DroneMode state = DroneMode::Stopped;
  double timeSecSinceStart = 0.0;
};

inline const char* toString(DroneMode mode)
{
  switch (mode) {
    case DroneMode::Stopped: return "Stopped";
    case DroneMode::Accelerating: return "Accelerating";
    case DroneMode::Decelerating: return "Decelerating";
    case DroneMode::Turning: return "Turning";
    case DroneMode::Moving: return "Moving";
  }
  return "Unknown";
}

class Homework10Error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

}  // namespace homework_10
