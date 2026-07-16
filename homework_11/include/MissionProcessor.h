#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "Types.h"
#include "io/GpioController.h"
#include "io/UartPort.h"

namespace homework_11 {

class MissionProcessor {
public:
  explicit MissionProcessor(RuntimeConfig config = {});

  static void installSignalHandlers();
  void init();
  void run();

private:
  void processIncomingByte(std::uint8_t byte);
  void handlePacket(std::uint8_t type, const std::uint8_t* payload, std::uint8_t len);
  void updateTargetTrack(const dlink::TargetPos& target);
  [[nodiscard]] std::optional<std::uint8_t> selectActiveTargetId() const;
  [[nodiscard]] std::optional<DropSolution> computeDropSolution() const;
  [[nodiscard]] GuidanceCommand computeGuidance() const;
  void sendControl(const GuidanceCommand& command);
  [[nodiscard]] bool shouldDropNow() const;
  void markActiveTargetDone(bool hitConfirmed);

  RuntimeConfig config_;
  UartPort uart_;
  GpioController gpio_;
  MissionSnapshot snapshot_{};
  dlink::Parser parser_{};
  std::array<std::uint8_t, 260> payload_{};
  std::optional<std::uint8_t> active_target_id_{};
  std::optional<std::uint32_t> drop_t_ms_{};
  bool waiting_result_ = false;
};

}  // namespace homework_11
