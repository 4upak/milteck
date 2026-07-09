#pragma once

#include <atomic>
#include <mutex>

#include "Types.h"
#include "utils/ThreadSafeQueue.h"

namespace homework_10 {

class DronePhysics {
public:
  DronePhysics(Coord initialPosition, double attackSpeed, double physicsTimeStep, double timeScale);
  ~DronePhysics();

  [[nodiscard]] bool isThreadReady() const;
  void run();
  void start();
  void stop();

  void pushCommand(const DroneCommand& command);
  [[nodiscard]] DroneTelemetry getTelemetry() const;
  void stepOnce(double dt);

private:
  void applyCommands();
  void setSpeedVector(double scalarSpeed);

  double attack_speed_ = 0.0;
  double physics_time_step_ = 0.01;
  double time_scale_ = 10.0;
  double scalar_speed_ = 0.0;
  double acceleration_ = 10.0;
  double desired_direction_ = 0.0;
  double command_angle_speed_ = 1.0;

  mutable std::mutex telemetry_mutex_;
  DroneTelemetry telemetry_{};
  ThreadSafeQueue<DroneCommand> commands_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> started_{false};
  std::atomic<bool> stop_requested_{false};
};

}  // namespace homework_10
