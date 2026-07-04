#pragma once

#include <string>

namespace homework_11 {

class GpioController {
public:
  GpioController() = default;
  ~GpioController();

  GpioController(const GpioController&) = delete;
  GpioController& operator=(const GpioController&) = delete;

  void open(const std::string& chipName, unsigned startLine, unsigned dropLine, bool dryRun = false);
  void close();
  void setStartReady();
  void resetOutputs();
  void pulseDrop(unsigned microseconds = 80000);
  [[nodiscard]] bool isDryRun() const { return dry_run_; }

private:
  std::string chip_name_;
  unsigned start_line_ = 0;
  unsigned drop_line_ = 0;
  bool dry_run_ = false;
  void* chip_ = nullptr;
  void* start_ = nullptr;
  void* drop_ = nullptr;
};

}  // namespace homework_11
