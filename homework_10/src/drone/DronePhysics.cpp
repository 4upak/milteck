#include "drone/DronePhysics.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "drone/DroneState.h"

namespace homework_10 {

DronePhysics::DronePhysics(Coord initialPosition, double attackSpeed, double physicsTimeStep, double timeScale)
  : attack_speed_(attackSpeed)
  , physics_time_step_(physicsTimeStep > 0.0 ? physicsTimeStep : 0.01)
  , time_scale_(timeScale > 0.0 ? timeScale : 1.0)
  , acceleration_(std::max(1.0, attackSpeed * 2.0))
{
  telemetry_.pos = initialPosition;
  telemetry_.state = DroneMode::Stopped;
}

DronePhysics::~DronePhysics()
{
  stop();
}

bool DronePhysics::isThreadReady() const
{
  return ready_.load();
}

void DronePhysics::run()
{
  ready_.store(true);
  while (!stop_requested_.load() && !started_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  while (!stop_requested_.load()) {
    stepOnce(physics_time_step_);
    std::this_thread::sleep_for(std::chrono::duration<double>(physics_time_step_ / time_scale_));
  }
}

void DronePhysics::start()
{
  started_.store(true);
}

void DronePhysics::stop()
{
  stop_requested_.store(true);
}

void DronePhysics::pushCommand(const DroneCommand& command)
{
  commands_.push(command);
}

DroneTelemetry DronePhysics::getTelemetry() const
{
  std::lock_guard<std::mutex> lock(telemetry_mutex_);
  return telemetry_;
}

void DronePhysics::stepOnce(double dt)
{
  applyCommands();

  std::lock_guard<std::mutex> lock(telemetry_mutex_);
  const double turn_speed = command_angle_speed_ > 0.0 ? command_angle_speed_ : 1.0;
  const double delta = normalizeAngle(desired_direction_ - telemetry_.direction);

  if (telemetry_.state == DroneMode::Turning || std::fabs(delta) > 0.01) {
    const double max_turn = turn_speed * dt;
    if (std::fabs(delta) <= max_turn) {
      telemetry_.direction = desired_direction_;
      telemetry_.state = scalar_speed_ > 0.01 ? DroneMode::Moving : DroneMode::Accelerating;
    }
    else {
      telemetry_.direction = normalizeAngle(telemetry_.direction + (delta > 0.0 ? max_turn : -max_turn));
      telemetry_.state = DroneMode::Turning;
    }
  }

  if (telemetry_.state == DroneMode::Accelerating || telemetry_.state == DroneMode::Moving) {
    scalar_speed_ = std::min(attack_speed_, scalar_speed_ + acceleration_ * dt);
    if (scalar_speed_ >= attack_speed_ - 1e-6) {
      telemetry_.state = DroneMode::Moving;
    }
  }
  else if (telemetry_.state == DroneMode::Decelerating) {
    scalar_speed_ = std::max(0.0, scalar_speed_ - acceleration_ * dt);
    if (scalar_speed_ <= 1e-6) {
      telemetry_.state = DroneMode::Stopped;
    }
  }
  else if (telemetry_.state == DroneMode::Stopped) {
    scalar_speed_ = 0.0;
  }

  telemetry_.pos.x += std::cos(telemetry_.direction) * scalar_speed_ * dt;
  telemetry_.pos.y += std::sin(telemetry_.direction) * scalar_speed_ * dt;
  setSpeedVector(scalar_speed_);
  telemetry_.timeSecSinceStart += dt;
}

void DronePhysics::applyCommands()
{
  while (auto command = commands_.tryPop()) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    telemetry_.state = command->state;
    desired_direction_ = command->desiredDirection;
    command_angle_speed_ = command->angleSpeed;
    if (command->targetSpeed > 0.0) {
      attack_speed_ = command->targetSpeed;
    }
  }
}

void DronePhysics::setSpeedVector(double scalarSpeed)
{
  telemetry_.speed = Coord{std::cos(telemetry_.direction) * scalarSpeed, std::sin(telemetry_.direction) * scalarSpeed};
}

}  // namespace homework_10
