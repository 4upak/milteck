#pragma once

#include "fc_link.hpp"
#include "udp_socket.hpp"

#include <fstream>
#include <string>

class C2Controller {
public:
    enum class C2State {
        DISARMED,
        ARMED_HOLD,
        ARMED_GUIDED,
        ARMED_MANUAL,
    };

    C2Controller(FcLink& fc, UdpSocket& waypoints, std::ostream& log);

    void tick();
    [[nodiscard]] C2State current_state() const;

private:
    FcLink& fc_;
    UdpSocket& waypoints_;
    std::ostream& log_;
    C2State state_{C2State::DISARMED};
    bool health_marked_{false};

    void transition(C2State next);
    void update_state_from_fc();
    void handle_waypoint(const std::string& json);
    void mark_healthy_once();

    static const char* state_name(C2State state);
};
