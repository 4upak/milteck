#include "fc_link.hpp"

#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace {
constexpr uint8_t kMavlink2Magic = 0xFD;
constexpr uint8_t kThisSystemId = 245;
constexpr uint8_t kThisComponentId = 190;
constexpr uint8_t kMavModeFlagSafetyArmed = 128;
constexpr uint8_t kMavModeFlagCustomModeEnabled = 1;
constexpr uint8_t kMavFrameLocalNed = 1;
constexpr uint16_t kMavCmdDoSetMode = 176;
constexpr uint32_t kMsgHeartbeat = 0;
constexpr uint32_t kMsgCommandLong = 76;
constexpr uint32_t kMsgSetPositionTargetLocalNed = 84;
constexpr uint8_t kCrcExtraCommandLong = 152;
constexpr uint8_t kCrcExtraSetPositionTargetLocalNed = 143;

uint16_t crc_accumulate(uint8_t data, uint16_t crc) {
    data ^= static_cast<uint8_t>(crc & 0xffU);
    data ^= static_cast<uint8_t>(data << 4U);
    return static_cast<uint16_t>(
        (crc >> 8U) ^ (static_cast<uint16_t>(data) << 8U) ^
        (static_cast<uint16_t>(data) << 3U) ^ (static_cast<uint16_t>(data) >> 4U));
}

uint16_t crc_calculate(const uint8_t* data, std::size_t len, uint8_t crc_extra) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc = crc_accumulate(data[i], crc);
    }
    crc = crc_accumulate(crc_extra, crc);
    return crc;
}

template <typename T>
void put_le(std::array<uint8_t, 255>& payload, std::size_t offset, T value) {
    static_assert(std::is_integral_v<T> || std::is_floating_point_v<T>);
    uint8_t bytes[sizeof(T)]{};
    std::memcpy(bytes, &value, sizeof(T));
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        payload[offset + i] = bytes[i];
    }
}

uint32_t read_u32_le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8U) |
           (static_cast<uint32_t>(p[2]) << 16U) |
           (static_cast<uint32_t>(p[3]) << 24U);
}
}

FcLink::FcLink(uint16_t listen_port) {
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        throw std::runtime_error(std::string("FC socket failed: ") + std::strerror(errno));
    }

    int yes = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listen_port);
    if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        throw std::runtime_error(std::string("FC bind failed: ") + std::strerror(errno));
    }

    int flags = ::fcntl(sock_, F_GETFL, 0);
    if (flags >= 0) {
        ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
    }
}

FcLink::~FcLink() {
    if (sock_ >= 0) {
        ::close(sock_);
    }
}

void FcLink::poll() {
    std::array<uint8_t, 2048> buf{};
    while (true) {
        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        ssize_t n = ::recvfrom(sock_, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            throw std::runtime_error(std::string("FC recvfrom failed: ") + std::strerror(errno));
        }
        if (n > 0) {
            handle_packet(buf.data(), static_cast<std::size_t>(n), peer);
        }
    }
}

bool FcLink::is_connected() const { return connected_; }
bool FcLink::is_armed() const { return armed_; }
std::string FcLink::flight_mode() const { return mode_from_custom(custom_mode_); }

void FcLink::hold() {
    std::array<uint8_t, 255> payload{};
    put_le<float>(payload, 0, static_cast<float>(kMavModeFlagCustomModeEnabled));
    put_le<float>(payload, 4, 4.0F); // ArduRover HOLD custom mode.
    put_le<float>(payload, 8, 0.0F);
    put_le<float>(payload, 12, 0.0F);
    put_le<float>(payload, 16, 0.0F);
    put_le<float>(payload, 20, 0.0F);
    put_le<float>(payload, 24, 0.0F);
    put_le<uint16_t>(payload, 28, kMavCmdDoSetMode);
    payload[30] = 1; // target system, common SITL default
    payload[31] = 1; // target component
    payload[32] = 0; // confirmation
    send_mavlink2(kMsgCommandLong, payload.data(), 33, kCrcExtraCommandLong);
}

void FcLink::go_to_ned(double north_m, double east_m) {
    std::array<uint8_t, 255> payload{};
    put_le<uint32_t>(payload, 0, 0); // time_boot_ms; FC can accept 0 for setpoint stream samples.
    put_le<float>(payload, 4, static_cast<float>(north_m));
    put_le<float>(payload, 8, static_cast<float>(east_m));
    put_le<float>(payload, 12, 0.0F); // down
    put_le<float>(payload, 16, 0.0F); // vx
    put_le<float>(payload, 20, 0.0F); // vy
    put_le<float>(payload, 24, 0.0F); // vz
    put_le<float>(payload, 28, 0.0F); // afx
    put_le<float>(payload, 32, 0.0F); // afy
    put_le<float>(payload, 36, 0.0F); // afz
    put_le<float>(payload, 40, 0.0F); // yaw
    put_le<float>(payload, 44, 0.0F); // yaw_rate
    put_le<uint16_t>(payload, 48, 0x0DF8); // use position, ignore vel/accel/yaw/yaw_rate
    payload[50] = 1; // target_system
    payload[51] = 1; // target_component
    payload[52] = kMavFrameLocalNed;
    send_mavlink2(kMsgSetPositionTargetLocalNed, payload.data(), 53, kCrcExtraSetPositionTargetLocalNed);
}

void FcLink::handle_packet(const uint8_t* data, std::size_t size, const sockaddr_in& peer) {
    for (std::size_t i = 0; i + 12 <= size; ++i) {
        if (data[i] != kMavlink2Magic) {
            continue;
        }
        const uint8_t payload_len = data[i + 1];
        const std::size_t frame_len = 10U + payload_len + 2U;
        if (i + frame_len > size) {
            continue;
        }
        const uint32_t msg_id = static_cast<uint32_t>(data[i + 7]) |
                                (static_cast<uint32_t>(data[i + 8]) << 8U) |
                                (static_cast<uint32_t>(data[i + 9]) << 16U);
        const uint8_t* payload = data + i + 10;
        if (msg_id == kMsgHeartbeat && payload_len >= 9) {
            custom_mode_ = read_u32_le(payload);
            const uint8_t base_mode = payload[6];
            armed_ = (base_mode & kMavModeFlagSafetyArmed) != 0;
            connected_ = true;
            last_fc_addr_ = peer;
            have_fc_addr_ = true;
        }
        i += frame_len - 1U;
    }
}

void FcLink::send_mavlink2(uint32_t msg_id, const uint8_t* payload, uint8_t payload_len, uint8_t crc_extra) {
    if (!have_fc_addr_) {
        return;
    }

    std::array<uint8_t, 280> frame{};
    frame[0] = kMavlink2Magic;
    frame[1] = payload_len;
    frame[2] = 0; // incompat flags
    frame[3] = 0; // compat flags
    frame[4] = seq_++;
    frame[5] = kThisSystemId;
    frame[6] = kThisComponentId;
    frame[7] = static_cast<uint8_t>(msg_id & 0xffU);
    frame[8] = static_cast<uint8_t>((msg_id >> 8U) & 0xffU);
    frame[9] = static_cast<uint8_t>((msg_id >> 16U) & 0xffU);
    std::memcpy(frame.data() + 10, payload, payload_len);

    const uint16_t crc = crc_calculate(frame.data() + 1, 9U + payload_len, crc_extra);
    frame[10 + payload_len] = static_cast<uint8_t>(crc & 0xffU);
    frame[11 + payload_len] = static_cast<uint8_t>((crc >> 8U) & 0xffU);

    ::sendto(sock_, frame.data(), 12U + payload_len, 0, reinterpret_cast<sockaddr*>(&last_fc_addr_), sizeof(last_fc_addr_));
}

std::string FcLink::mode_from_custom(uint32_t custom_mode) const {
    switch (custom_mode) {
        case 0: return "Manual";
        case 4: return "Hold";
        case 15: return "Guided";
        default: return "Mode" + std::to_string(custom_mode);
    }
}
