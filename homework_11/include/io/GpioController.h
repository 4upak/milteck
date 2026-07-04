#pragma once

#include <string>

namespace homework_11 {

class GpioController {
public:
  GpioController() = default;
  ~GpioController();

  GpioController(const GpioController&) = delete;
  GpioController& operator=(const GpioController&) = delete;

  void open(const std::string& chipName, unsigned startLine, unsigned dropLine);
  void close();
  void setStartReady();
  void resetOutputs();
  void pulseDrop(unsigned microseconds = 80000);

private:
  std::string chip_name_;
  unsigned start_line_ = 0;
  unsigned drop_line_ = 0;
  void* chip_ = nullptr;
};

}  // namespace homework_11
