#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace homework_11 {

class UartPort {
public:
  UartPort() = default;
  ~UartPort();

  UartPort(const UartPort&) = delete;
  UartPort& operator=(const UartPort&) = delete;

  void open(const std::string& device, int baud = 115200);
  void close();
  [[nodiscard]] bool isOpen() const;
  [[nodiscard]] int fd() const;
  std::size_t readAvailable(std::vector<std::uint8_t>& out);
  void writeAll(const std::uint8_t* data, std::size_t size);

private:
  int fd_ = -1;
};

}  // namespace homework_11
