#include "drone/DroneState.h"

#include <cmath>
#include <memory>
#include <numbers>

namespace homework_09 {
namespace {

constexpr double kEpsilon = 1e-9;

[[nodiscard]] double positive_or_default(double value, double fallback)
{
  return value > kEpsilon ? value : fallback;
}

}  // namespace

double normalizeAngle(double angle)
{
  const double two_pi = 2.0 * std::numbers::pi;
  while (angle > std::numbers::pi) {
    angle -= two_pi;
  }
  while (angle < -std::numbers::pi) {
    angle += two_pi;
  }
  return angle;
}

bool needsTurn(const DroneContext& ctx)
{
  return std::fabs(normalizeAngle(ctx.desired_direction - ctx.direction)) > ctx.cfg.turn_threshold;
}

void advanceForward(DroneContext& ctx, double distance)
{
  ctx.position.x += distance * std::cos(ctx.direction);
  ctx.position.y += distance * std::sin(ctx.direction);
}

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx)
{
  const double delta = normalizeAngle(ctx.desired_direction - ctx.direction);
  if (std::fabs(delta) > ctx.cfg.turn_threshold) {
    ctx.target_direction = ctx.desired_direction;
    ctx.turn_remaining = std::fabs(delta) / positive_or_default(ctx.cfg.angular_speed, 1.0);
    return std::make_unique<StateTurning>();
  }

  ctx.direction = ctx.desired_direction;
  return std::make_unique<StateAccelerating>();
}

const char* StateStopped::name() const
{
  return "Stopped";
}

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx)
{
  if (needsTurn(ctx) && ctx.speed > kEpsilon) {
    return std::make_unique<StateDecelerating>();
  }
  if (needsTurn(ctx)) {
    return std::make_unique<StateTurning>();
  }

  const double dt = positive_or_default(ctx.cfg.sim_time_step, 1.0);
  const double accel = positive_or_default(ctx.cfg.acceleration, 1.0);
  const double max_speed = positive_or_default(ctx.cfg.attack_speed, ctx.speed);
  const double next_speed = std::min(max_speed, ctx.speed + accel * dt);
  advanceForward(ctx, 0.5 * (ctx.speed + next_speed) * dt);
  ctx.speed = next_speed;

  if (ctx.speed >= max_speed - kEpsilon) {
    return std::make_unique<StateMoving>();
  }
  return nullptr;
}

const char* StateAccelerating::name() const
{
  return "Accelerating";
}

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx)
{
  const double dt = positive_or_default(ctx.cfg.sim_time_step, 1.0);
  const double accel = positive_or_default(ctx.cfg.acceleration, 1.0);
  const double next_speed = std::max(0.0, ctx.speed - accel * dt);
  advanceForward(ctx, 0.5 * (ctx.speed + next_speed) * dt);
  ctx.speed = next_speed;

  if (ctx.speed <= kEpsilon) {
    ctx.speed = 0.0;
    return std::make_unique<StateStopped>();
  }
  return nullptr;
}

const char* StateDecelerating::name() const
{
  return "Decelerating";
}

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx)
{
  const double dt = positive_or_default(ctx.cfg.sim_time_step, 1.0);
  const double max_turn = positive_or_default(ctx.cfg.angular_speed, 1.0) * dt;
  const double delta = normalizeAngle(ctx.target_direction - ctx.direction);

  if (std::fabs(delta) <= max_turn || std::fabs(delta) <= ctx.cfg.turn_threshold) {
    ctx.direction = ctx.target_direction;
    ctx.turn_remaining = 0.0;
    return std::make_unique<StateAccelerating>();
  }

  ctx.direction = normalizeAngle(ctx.direction + (delta > 0.0 ? max_turn : -max_turn));
  ctx.turn_remaining = std::max(0.0, (std::fabs(delta) - max_turn) / positive_or_default(ctx.cfg.angular_speed, 1.0));
  return nullptr;
}

const char* StateTurning::name() const
{
  return "Turning";
}

std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx)
{
  if (needsTurn(ctx)) {
    return std::make_unique<StateDecelerating>();
  }

  const double dt = positive_or_default(ctx.cfg.sim_time_step, 1.0);
  const double speed = positive_or_default(ctx.cfg.attack_speed, ctx.speed);
  ctx.speed = speed;
  advanceForward(ctx, speed * dt);
  return nullptr;
}

const char* StateMoving::name() const
{
  return "Moving";
}

}  // namespace homework_09
