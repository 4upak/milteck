#pragma once

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>

class UdpSocket {
public:
    explicit UdpSocket(uint16_t listen_port) {
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
        }

        int yes = 1;
        ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(listen_port);
        if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error(std::string("bind UDP port failed: ") + std::strerror(errno));
        }

        int flags = ::fcntl(sock_, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
        }
    }

    ~UdpSocket() {
        if (sock_ >= 0) {
            ::close(sock_);
        }
    }

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    std::optional<std::string> receive_text() {
        std::array<char, 4096> buf{};
        ssize_t n = ::recv(sock_, buf.data(), buf.size() - 1, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return std::nullopt;
            }
            throw std::runtime_error(std::string("recv failed: ") + std::strerror(errno));
        }
        if (n == 0) {
            return std::nullopt;
        }
        return std::string(buf.data(), static_cast<std::size_t>(n));
    }

private:
    int sock_{-1};
};
