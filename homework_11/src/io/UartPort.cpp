#include "io/UartPort.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

namespace homework_11 {

namespace {

speed_t toSpeed(int baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 57600: return B57600;
    case 9600: return B9600;
    default: throw std::runtime_error("unsupported baud rate");
  }
}

}  // namespace

UartPort::~UartPort() { close(); }

void UartPort::open(const std::string& device, int baud)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "open uart failed");
  }

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    const int err = errno;
    close();
    throw std::system_error(err, std::generic_category(), "tcgetattr failed");
  }

  cfmakeraw(&tio);
  const speed_t speed = toSpeed(baud);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  tio.c_cflag |= (CLOCAL | CREAD);

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    const int err = errno;
    close();
    throw std::system_error(err, std::generic_category(), "tcsetattr failed");
  }
}

void UartPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool UartPort::isOpen() const { return fd_ >= 0; }
int UartPort::fd() const { return fd_; }

std::size_t UartPort::readAvailable(std::vector<std::uint8_t>& out)
{
  if (fd_ < 0) {
    return 0;
  }
  std::uint8_t buffer[256];
  const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
  if (n > 0) {
    out.insert(out.end(), buffer, buffer + n);
    return static_cast<std::size_t>(n);
  }
  if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    throw std::system_error(errno, std::generic_category(), "uart read failed");
  }
  return 0;
}

void UartPort::writeAll(const std::uint8_t* data, std::size_t size)
{
  std::size_t sent = 0;
  while (sent < size) {
    const ssize_t n = ::write(fd_, data + sent, size - sent);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      usleep(1000);
      continue;
    }
    throw std::system_error(errno, std::generic_category(), "uart write failed");
  }
}

}  // namespace homework_11
