#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "Types.h"

namespace homework_10 {

class ITargetProvider {
public:
  virtual ~ITargetProvider() = default;

  ITargetProvider() = default;
  ITargetProvider(const ITargetProvider&) = delete;
  ITargetProvider& operator=(const ITargetProvider&) = delete;
  ITargetProvider(ITargetProvider&&) = delete;
  ITargetProvider& operator=(ITargetProvider&&) = delete;

  [[nodiscard]] virtual bool isThreadReady() const = 0;
  virtual void run() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;

  [[nodiscard]] virtual std::size_t getTargetCount() const = 0;
  [[nodiscard]] virtual Target getTarget(std::size_t index) const = 0;
  [[nodiscard]] virtual std::vector<Target> targets() const = 0;
  [[nodiscard]] virtual Target findByName(const std::string& name) const = 0;
};

}  // namespace homework_10
