#pragma once

#include <memory>

#include "Types.h"

namespace homework_09 {

struct DroneMotionConfig {
  double attack_speed = 0.0;
  double acceleration = 1.0;
  double angular_speed = 1.0;
  double sim_time_step = 1.0;
  double turn_threshold = 0.01;
};

struct DroneContext {
  Coord position{};
  double speed = 0.0;
  double direction = 0.0;
  double desired_direction = 0.0;
  double target_direction = 0.0;
  double turn_remaining = 0.0;
  DroneMotionConfig cfg{};
};

class IDroneState {
public:
  virtual ~IDroneState() = default;

  IDroneState() = default;
  IDroneState(const IDroneState&) = delete;
  IDroneState& operator=(const IDroneState&) = delete;
  IDroneState(IDroneState&&) = delete;
  IDroneState& operator=(IDroneState&&) = delete;

  [[nodiscard]] virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx) = 0;
  [[nodiscard]] virtual const char* name() const = 0;
};

class StateStopped : public IDroneState {
public:
  [[nodiscard]] std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
  [[nodiscard]] const char* name() const override;
};

class StateAccelerating : public IDroneState {
public:
  [[nodiscard]] std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
  [[nodiscard]] const char* name() const override;
};

class StateDecelerating : public IDroneState {
public:
  [[nodiscard]] std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
  [[nodiscard]] const char* name() const override;
};

class StateTurning : public IDroneState {
public:
  [[nodiscard]] std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
  [[nodiscard]] const char* name() const override;
};

class StateMoving : public IDroneState {
public:
  [[nodiscard]] std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
  [[nodiscard]] const char* name() const override;
};

[[nodiscard]] double normalizeAngle(double angle);
[[nodiscard]] bool needsTurn(const DroneContext& ctx);
void advanceForward(DroneContext& ctx, double distance);

}  // namespace homework_09
