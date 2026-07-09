#pragma once

#include <mutex>
#include <optional>
#include <queue>

namespace homework_10 {

template <typename T>
class ThreadSafeQueue {
public:
  void push(T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(std::move(value));
  }

  [[nodiscard]] std::optional<T> tryPop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }
    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

  [[nodiscard]] bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  [[nodiscard]] std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

private:
  mutable std::mutex mutex_;
  std::queue<T> queue_;
};

}  // namespace homework_10
