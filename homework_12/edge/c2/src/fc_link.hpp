#pragma once

#include <cstdint>
#include <netinet/in.h>
#include <optional>
#include <string>

class FcLink {
public:
    explicit FcLink(uint16_t listen_port);
    ~FcLink();

    FcLink(const FcLink&) = delete;
    FcLink& operator=(const FcLink&) = delete;

    void poll();

    [[nodiscard]] bool is_connected() const;
    [[nodiscard]] bool is_armed() const;
    [[nodiscard]] std::string flight_mode() const;

    void hold();
    void go_to_ned(double north_m, double east_m);

private:
    int sock_{-1};
    sockaddr_in last_fc_addr_{};
    bool have_fc_addr_{false};
    bool connected_{false};
    bool armed_{false};
    uint32_t custom_mode_{0};
    uint8_t seq_{0};

    void handle_packet(const uint8_t* data, std::size_t size, const sockaddr_in& peer);
    void send_mavlink2(uint32_t msg_id, const uint8_t* payload, uint8_t payload_len, uint8_t crc_extra);
    [[nodiscard]] std::string mode_from_custom(uint32_t custom_mode) const;
};
