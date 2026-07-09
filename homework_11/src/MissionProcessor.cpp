#include "MissionProcessor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <thread>
#include <vector>
#include <csignal>

namespace homework_11 {

namespace {

volatile std::sig_atomic_t g_stopRequested = 0;

void signalHandler(int)
{
  g_stopRequested = 1;
}

constexpr double kGravity = 9.81;
constexpr double kEpsilon = 1e-9;
constexpr double kMaxAbsCommand = 1.0;
constexpr std::uint32_t kResultTimeoutMs = 5000;

[[nodiscard]] double clampValue(double value, double low, double high)
{
  return std::clamp(value, low, high);
}

[[nodiscard]] float clampUnit(double value)
{
  return static_cast<float>(clampValue(value, -kMaxAbsCommand, kMaxAbsCommand));
}

[[nodiscard]] double positiveOrDefault(double value, double fallback)
{
  return value > kEpsilon ? value : fallback;
}

[[nodiscard]] double norm(Coord value)
{
  return std::hypot(value.x, value.y);
}

[[nodiscard]] double distance(Coord a, Coord b)
{
  return norm(a - b);
}

[[nodiscard]] double directionTo(Coord from, Coord to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

[[nodiscard]] double normalizeAngle(double angle)
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

[[nodiscard]] Coord telemetryPosition(const dlink::Telemetry& t)
{
  return Coord{t.x, t.y};
}

[[nodiscard]] Coord forwardUnit(double direction)
{
  return Coord{std::cos(direction), std::sin(direction)};
}

[[nodiscard]] double solveCardanoTime(double cubicA, double cubicB, double cubicC)
{
  if (std::fabs(cubicA) < kEpsilon) {
    throw Homework11Error{"flight time equation degenerates (a ~ 0)"};
  }

  const double depressedP = -(cubicB * cubicB) / (3.0 * cubicA * cubicA);
  const double depressedQ = (2.0 * cubicB * cubicB * cubicB) / (27.0 * cubicA * cubicA * cubicA) + cubicC / cubicA;
  if (depressedP >= 0.0) {
    throw Homework11Error{"flight time equation has no real positive root (p >= 0)"};
  }

  const double acosArg = (3.0 * depressedQ / (2.0 * depressedP)) * std::sqrt(-3.0 / depressedP);
  if (acosArg < -1.0 || acosArg > 1.0) {
    throw Homework11Error{"flight time equation has complex roots"};
  }

  const double phi = std::acos(acosArg);
  const double rootBase = 2.0 * std::sqrt(-depressedP / 3.0);
  const double offset = cubicB / (3.0 * cubicA);
  const double twoPi = 2.0 * std::numbers::pi;

  const double roots[3] = {
    rootBase * std::cos(phi / 3.0) - offset,
    rootBase * std::cos((phi + twoPi) / 3.0) - offset,
    rootBase * std::cos((phi + 2.0 * twoPi) / 3.0) - offset,
  };

  double smallestPositive = -1.0;
  for (double candidate : roots) {
    if (candidate > kEpsilon && (smallestPositive < 0.0 || candidate < smallestPositive)) {
      smallestPositive = candidate;
    }
  }
  if (smallestPositive < 0.0) {
    throw Homework11Error{"flight time equation has no positive root"};
  }
  return smallestPositive;
}

[[nodiscard]] double calcHorizontalDistance(double elapsedTime, const dlink::AmmoCfg& ammo, double attackSpeed)
{
  const double mass = ammo.mass;
  const double drag = ammo.drag;
  const double lift = ammo.lift;
  const double initialSpeed = attackSpeed;

  const double term1 = initialSpeed * elapsedTime;
  const double term2 = -(elapsedTime * elapsedTime * drag * initialSpeed) / (2.0 * mass);
  const double term3 = (elapsedTime * elapsedTime * elapsedTime *
                        (6.0 * drag * kGravity * lift * mass - 6.0 * drag * drag * (lift * lift - 1.0) * initialSpeed)) /
                       (36.0 * mass * mass);
  const double term4 =
    (std::pow(elapsedTime, 4.0) * (-6.0 * drag * drag * kGravity * lift * (1.0 + lift * lift + lift * lift * lift * lift) * mass +
                                   3.0 * drag * drag * drag * lift * lift * (1.0 + lift * lift) * initialSpeed +
                                   6.0 * drag * drag * drag * lift * lift * lift * lift * (1.0 + lift * lift) * initialSpeed)) /
    (36.0 * std::pow(1.0 + lift * lift, 2.0) * mass * mass * mass);
  const double term5 =
    (std::pow(elapsedTime, 5.0) * (3.0 * drag * drag * drag * kGravity * lift * lift * lift * mass -
                                   3.0 * std::pow(drag, 4.0) * lift * lift * (1.0 + lift * lift) * initialSpeed)) /
    (36.0 * (1.0 + lift * lift) * std::pow(mass, 4.0));

  return term1 + term2 + term3 + term4 + term5;
}

}  // namespace

MissionProcessor::MissionProcessor(RuntimeConfig config)
    : config_(std::move(config))
{
}

void MissionProcessor::installSignalHandlers()
{
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
}

void MissionProcessor::init()
{
  uart_.open(config_.uart_device, 115200);
  gpio_.open(config_.gpio_chip, config_.start_line, config_.drop_line);
  gpio_.setStartReady();
}

void MissionProcessor::run()
{
  std::vector<std::uint8_t> incoming;
  incoming.reserve(256);

  for (; !g_stopRequested;) {
    incoming.clear();
    uart_.readAvailable(incoming);
    for (std::uint8_t byte : incoming) {
      processIncomingByte(byte);
    }

    if (waiting_result_ && snapshot_.telemetry && drop_t_ms_.has_value()) {
      const std::uint32_t now = snapshot_.telemetry->t_ms;
      if (now >= *drop_t_ms_ && now - *drop_t_ms_ > kResultTimeoutMs) {
        markActiveTargetDone(false);
      }
    }

    const GuidanceCommand command = computeGuidance();
    sendControl(command);

    if (!waiting_result_ && shouldDropNow()) {
      gpio_.pulseDrop(config_.drop_hold_us);
      waiting_result_ = true;
      if (snapshot_.telemetry) {
        drop_t_ms_ = snapshot_.telemetry->t_ms;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(config_.control_period_ms));
  }

  gpio_.resetOutputs();
}

void MissionProcessor::processIncomingByte(std::uint8_t byte)
{
  std::uint8_t type = 0;
  std::uint8_t len = 0;
  if (parser_.feed(byte, type, payload_.data(), len)) {
    handlePacket(type, payload_.data(), len);
  }
}

void MissionProcessor::handlePacket(std::uint8_t type, const std::uint8_t* payload, std::uint8_t len)
{
  switch (type) {
    case dlink::PKT_TELEMETRY:
      if (len == sizeof(dlink::Telemetry)) {
        dlink::Telemetry telemetry{};
        std::memcpy(&telemetry, payload, sizeof(telemetry));
        snapshot_.telemetry = telemetry;
      }
      break;
    case dlink::PKT_TARGET:
      if (len == sizeof(dlink::TargetPos)) {
        dlink::TargetPos target{};
        std::memcpy(&target, payload, sizeof(target));
        updateTargetTrack(target);
      }
      break;
    case dlink::PKT_AMMO:
      if (len == sizeof(dlink::AmmoCfg)) {
        dlink::AmmoCfg ammo{};
        std::memcpy(&ammo, payload, sizeof(ammo));
        snapshot_.ammo = ammo;
      }
      break;
    case dlink::PKT_CONFIG:
      if (len == sizeof(dlink::DroneCfg)) {
        dlink::DroneCfg config{};
        std::memcpy(&config, payload, sizeof(config));
        snapshot_.config = config;
      }
      break;
    case dlink::PKT_RESULT:
      if (len == sizeof(dlink::Result)) {
        dlink::Result result{};
        std::memcpy(&result, payload, sizeof(result));
        snapshot_.result = result;
        markActiveTargetDone(result.hit != 0);
      }
      break;
    default:
      break;
  }
}

void MissionProcessor::updateTargetTrack(const dlink::TargetPos& target)
{
  if (target.id >= snapshot_.targets.size()) {
    return;
  }
  TargetTrack& track = snapshot_.targets[target.id];
  const Coord newPos{target.x, target.y};
  if (track.seen && snapshot_.telemetry) {
    const std::uint32_t dtMs = snapshot_.telemetry->t_ms - track.t_ms;
    if (dtMs > 0) {
      const double dt = static_cast<double>(dtMs) / 1000.0;
      track.velocity = (newPos - track.pos) * (1.0 / dt);
    }
  }
  track.seen = true;
  track.id = target.id;
  track.pos = newPos;
  track.t_ms = snapshot_.telemetry ? snapshot_.telemetry->t_ms : track.t_ms;
  if (!active_target_id_.has_value()) {
    active_target_id_ = target.id;
  }
}

std::optional<std::uint8_t> MissionProcessor::selectActiveTargetId() const
{
  const std::size_t limit = snapshot_.ammo ? std::min<std::size_t>(snapshot_.ammo->nTargets, snapshot_.targets.size()) : snapshot_.targets.size();

  if (active_target_id_.has_value()) {
    const TargetTrack& current = snapshot_.targets[*active_target_id_];
    if (*active_target_id_ < limit && current.seen && !current.done) {
      return active_target_id_;
    }
  }

  for (std::size_t idx = 0; idx < limit; ++idx) {
    const TargetTrack& track = snapshot_.targets[idx];
    if (track.seen && !track.done) {
      return static_cast<std::uint8_t>(idx);
    }
  }
  return std::nullopt;
}

std::optional<DropSolution> MissionProcessor::computeDropSolution() const
{
  // PKT_CONFIG is optional: the checker may not send it in this homework.
  // When absent we fall back to telemetry-derived speed and sane defaults.
  if (!snapshot_.telemetry || !snapshot_.ammo) {
    return std::nullopt;
  }

  const std::optional<std::uint8_t> targetId = selectActiveTargetId();
  if (!targetId.has_value()) {
    return std::nullopt;
  }

  const TargetTrack& track = snapshot_.targets[*targetId];
  if (!track.seen || track.done) {
    return std::nullopt;
  }

  const dlink::Telemetry& telemetry = *snapshot_.telemetry;
  const dlink::AmmoCfg& ammo = *snapshot_.ammo;
  const dlink::DroneCfg config = snapshot_.config.value_or(dlink::DroneCfg{});
  const Coord dronePos = telemetryPosition(telemetry);
  const double altitude = positiveOrDefault(telemetry.z, 1.0);
  const double attackSpeed = positiveOrDefault(config.attackSpeed, positiveOrDefault(telemetry.speed, 1.0));

  if (ammo.mass <= 0.0F || altitude <= 0.0 || attackSpeed <= 0.0) {
    return std::nullopt;
  }

  const double cubicA = ammo.drag * kGravity * ammo.mass - 2.0 * ammo.drag * ammo.drag * ammo.lift * attackSpeed;
  const double cubicB = -3.0 * kGravity * ammo.mass * ammo.mass + 3.0 * ammo.drag * ammo.lift * ammo.mass * attackSpeed;
  const double cubicC = 6.0 * ammo.mass * ammo.mass * altitude;

  const double timeOfFlight = solveCardanoTime(cubicA, cubicB, cubicC);
  const double horizontalDistance = calcHorizontalDistance(timeOfFlight, ammo, attackSpeed);
  if (!std::isfinite(horizontalDistance) || horizontalDistance <= kEpsilon) {
    return std::nullopt;
  }

  const Coord predictedTarget = track.pos + track.velocity * timeOfFlight;
  const Coord targetDelta = predictedTarget - dronePos;
  const double currentDistance = norm(targetDelta);
  if (currentDistance <= kEpsilon) {
    return std::nullopt;
  }

  const double ratio = (currentDistance - horizontalDistance) / currentDistance;
  DropSolution solution;
  solution.valid = true;
  solution.target_now = track.pos;
  solution.predicted_target = predictedTarget;
  solution.drop_point = dronePos + targetDelta * ratio;
  solution.distance_to_drop = distance(dronePos, solution.drop_point);
  solution.distance_to_target = distance(dronePos, track.pos);
  solution.time_of_flight = timeOfFlight;
  solution.horizontal_distance = horizontalDistance;

  const Coord forward = forwardUnit(telemetry.dir);
  const Coord toPredicted = predictedTarget - dronePos;
  solution.lateral_error = std::fabs(forward.x * toPredicted.y - forward.y * toPredicted.x);
  solution.heading_error = normalizeAngle(directionTo(dronePos, solution.drop_point) - telemetry.dir);
  return solution;
}

GuidanceCommand MissionProcessor::computeGuidance() const
{
  const std::optional<DropSolution> solution = computeDropSolution();
  if (!solution.has_value() || !snapshot_.telemetry) {
    return GuidanceCommand{};
  }

  const dlink::Telemetry& telemetry = *snapshot_.telemetry;
  const dlink::DroneCfg config = snapshot_.config.value_or(dlink::DroneCfg{});
  const double headingError = solution->heading_error;
  const double absHeadingError = std::fabs(headingError);
  const double turnScale = std::max(positiveOrDefault(config.turnThreshold, 0.15) * 3.0, 0.35);
  const double distanceToDrop = solution->distance_to_drop;
  const double speed = positiveOrDefault(telemetry.speed, std::hypot(telemetry.vx, telemetry.vy));
  const double maxSpeed = positiveOrDefault(config.attackSpeed, std::max(speed, 1.0));
  const double accelPath = positiveOrDefault(config.accelerationPath, std::max(maxSpeed, 1.0));
  const double maxAccel = std::max((maxSpeed * maxSpeed) / (2.0 * accelPath), 0.5);
  const double brakingDistance = (speed * speed) / (2.0 * maxAccel);

  GuidanceCommand command;
  command.turnRate = clampUnit(headingError / turnScale);

  if (waiting_result_) {
    command.accel = -0.35F;
    return command;
  }

  if (absHeadingError > 1.0) {
    command.accel = -0.6F;
  }
  else if (absHeadingError > 0.45) {
    command.accel = speed > maxSpeed * 0.4 ? -0.2F : 0.1F;
  }
  else if (distanceToDrop > brakingDistance + solution->horizontal_distance * 0.15) {
    const double speedError = maxSpeed - speed;
    command.accel = clampUnit(speedError / std::max(maxSpeed, 1.0));
  }
  else {
    const double desiredSpeed = clampValue(distanceToDrop / std::max(positiveOrDefault(config.timeStep, 0.05), 0.05), 0.0, maxSpeed * 0.6);
    command.accel = clampUnit((desiredSpeed - speed) / std::max(maxSpeed, 1.0));
  }

  return command;
}

void MissionProcessor::sendControl(const GuidanceCommand& command)
{
  dlink::Control packet{command.accel, command.turnRate};
  std::uint8_t out[64]{};
  const std::size_t size = dlink::encode(dlink::PKT_CONTROL, &packet, sizeof(packet), out);
  uart_.writeAll(out, size);
}

bool MissionProcessor::shouldDropNow() const
{
  const std::optional<DropSolution> solution = computeDropSolution();
  if (!solution.has_value() || !snapshot_.telemetry || !snapshot_.ammo) {
    return false;
  }

  const dlink::Telemetry& telemetry = *snapshot_.telemetry;
  const dlink::AmmoCfg& ammo = *snapshot_.ammo;
  const dlink::DroneCfg config = snapshot_.config.value_or(dlink::DroneCfg{});
  const double dt = positiveOrDefault(config.timeStep, 0.05);
  const double dynamicTolerance = std::max<double>(ammo.hitRadius * 0.6, telemetry.speed * dt * 1.5);
  const double headingTolerance = std::max<double>(positiveOrDefault(config.turnThreshold, 0.15) * 1.5, 0.2);

  return solution->distance_to_drop <= dynamicTolerance &&
         std::fabs(solution->heading_error) <= headingTolerance &&
         solution->lateral_error <= ammo.hitRadius;
}

void MissionProcessor::markActiveTargetDone(bool hitConfirmed)
{
  const std::optional<std::uint8_t> targetId = active_target_id_.has_value() ? active_target_id_ : selectActiveTargetId();
  if (targetId.has_value() && *targetId < snapshot_.targets.size()) {
    snapshot_.targets[*targetId].done = true;
  }
  waiting_result_ = false;
  drop_t_ms_.reset();

  const std::size_t limit = snapshot_.ammo ? std::min<std::size_t>(snapshot_.ammo->nTargets, snapshot_.targets.size()) : snapshot_.targets.size();
  std::optional<std::uint8_t> next;
  for (std::size_t idx = 0; idx < limit; ++idx) {
    if (snapshot_.targets[idx].seen && !snapshot_.targets[idx].done) {
      next = static_cast<std::uint8_t>(idx);
      break;
    }
  }
  active_target_id_ = next;

  if (!hitConfirmed) {
    // even on miss we advance to the next target to avoid endlessly circling the same one
  }
}

}  // namespace homework_11
