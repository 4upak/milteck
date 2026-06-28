#pragma once

#include <atomic>
#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "interfaces/ITargetProvider.h"
#include "Types.h"

namespace homework_10 {

class ThreadSafeTargetProvider : public ITargetProvider {
public:
  ThreadSafeTargetProvider(std::string path, double arrayTimeStep, double timeScale);
  ~ThreadSafeTargetProvider() override;

  [[nodiscard]] bool isThreadReady() const override;
  void run() override;
  void start() override;
  void stop() override;

  [[nodiscard]] std::size_t getTargetCount() const override;
  [[nodiscard]] Target getTarget(std::size_t index) const override;
  [[nodiscard]] std::vector<Target> targets() const override;
  [[nodiscard]] Target findByName(const std::string& name) const override;

private:
  struct MovingTarget {
    std::string name;
    std::vector<Coord> trajectory;
    std::size_t current = 0;
    Target snapshot{};
  };

  void load(const std::string& path);
  void advanceOneStep();

  double array_time_step_ = 0.05;
  double time_scale_ = 10.0;
  mutable std::mutex mutex_;
  std::vector<MovingTarget> targets_;
  std::unordered_map<std::string, std::size_t> name_to_index_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> started_{false};
  std::atomic<bool> stop_requested_{false};
};

}  // namespace homework_10
