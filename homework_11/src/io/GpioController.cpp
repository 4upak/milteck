#include "io/GpioController.h"

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

#if defined(HOMEWORK11_HAS_GPIOD_H)
#include <gpiod.h>
#endif

namespace homework_11 {
namespace {

#if defined(HOMEWORK11_HAS_GPIOD_H)
std::string chipPathFromName(const std::string& chipName)
{
  if (chipName.rfind("/dev/", 0) == 0) {
    return chipName;
  }
  return "/dev/" + chipName;
}
#endif

}  // namespace

GpioController::~GpioController() { close(); }

void GpioController::open(const std::string& chipName, unsigned startLine, unsigned dropLine)
{
  close();
  chip_name_ = chipName;
  start_line_ = startLine;
  drop_line_ = dropLine;

#if defined(HOMEWORK11_HAS_GPIOD_H)
  const std::string chipPath = chipPathFromName(chip_name_);
  gpiod_chip* chip = gpiod_chip_open(chipPath.c_str());
  if (!chip) {
    throw std::runtime_error("failed to open gpio chip " + chipPath);
  }

  gpiod_line_settings* settings = gpiod_line_settings_new();
  gpiod_line_config* lineConfig = gpiod_line_config_new();
  gpiod_request_config* requestConfig = gpiod_request_config_new();
  if (!settings || !lineConfig || !requestConfig) {
    if (settings) gpiod_line_settings_free(settings);
    if (lineConfig) gpiod_line_config_free(lineConfig);
    if (requestConfig) gpiod_request_config_free(requestConfig);
    gpiod_chip_close(chip);
    throw std::runtime_error("failed to allocate libgpiod config objects");
  }

  if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT) != 0) {
    gpiod_line_settings_free(settings);
    gpiod_line_config_free(lineConfig);
    gpiod_request_config_free(requestConfig);
    gpiod_chip_close(chip);
    throw std::runtime_error("failed to set GPIO direction");
  }

  const unsigned int offsets[] = {start_line_, drop_line_};
  if (gpiod_line_config_add_line_settings(lineConfig, offsets, 2, settings) != 0) {
    gpiod_line_settings_free(settings);
    gpiod_line_config_free(lineConfig);
    gpiod_request_config_free(requestConfig);
    gpiod_chip_close(chip);
    throw std::runtime_error("failed to attach line settings");
  }

  const gpiod_line_value initialValues[] = {GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE};
  if (gpiod_line_config_set_output_values(lineConfig, initialValues, 2) != 0) {
    gpiod_line_settings_free(settings);
    gpiod_line_config_free(lineConfig);
    gpiod_request_config_free(requestConfig);
    gpiod_chip_close(chip);
    throw std::runtime_error("failed to set initial GPIO values");
  }

  gpiod_request_config_set_consumer(requestConfig, "drone");
  gpiod_line_request* request = gpiod_chip_request_lines(chip, requestConfig, lineConfig);

  gpiod_line_settings_free(settings);
  gpiod_line_config_free(lineConfig);
  gpiod_request_config_free(requestConfig);
  gpiod_chip_close(chip);

  if (!request) {
    throw std::runtime_error("failed to request GPIO lines");
  }

  chip_ = request;
#else
  throw std::runtime_error("libgpiod headers are unavailable in this build");
#endif
}

void GpioController::close()
{
  if (chip_) {
    try {
      resetOutputs();
    }
    catch (...) {
    }
  }
#if defined(HOMEWORK11_HAS_GPIOD_H)
  if (chip_) {
    gpiod_line_request_release(static_cast<gpiod_line_request*>(chip_));
  }
#endif
  chip_ = nullptr;
}

void GpioController::setStartReady()
{
#if defined(HOMEWORK11_HAS_GPIOD_H)
  if (gpiod_line_request_set_value(static_cast<gpiod_line_request*>(chip_), start_line_, GPIOD_LINE_VALUE_ACTIVE) != 0) {
    throw std::runtime_error("failed to set START line");
  }
#endif
}

void GpioController::resetOutputs()
{
#if defined(HOMEWORK11_HAS_GPIOD_H)
  auto* request = static_cast<gpiod_line_request*>(chip_);
  if (!request) {
    return;
  }
  if (gpiod_line_request_set_value(request, start_line_, GPIOD_LINE_VALUE_INACTIVE) != 0) {
    throw std::runtime_error("failed to reset START line");
  }
  if (gpiod_line_request_set_value(request, drop_line_, GPIOD_LINE_VALUE_INACTIVE) != 0) {
    throw std::runtime_error("failed to reset DROP line");
  }
#endif
}

void GpioController::pulseDrop(unsigned microseconds)
{
#if defined(HOMEWORK11_HAS_GPIOD_H)
  auto* request = static_cast<gpiod_line_request*>(chip_);
  if (gpiod_line_request_set_value(request, drop_line_, GPIOD_LINE_VALUE_ACTIVE) != 0) {
    throw std::runtime_error("failed to set DROP line high");
  }
  std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
  if (gpiod_line_request_set_value(request, drop_line_, GPIOD_LINE_VALUE_INACTIVE) != 0) {
    throw std::runtime_error("failed to set DROP line low");
  }
#else
  (void)microseconds;
#endif
}

}  // namespace homework_11
