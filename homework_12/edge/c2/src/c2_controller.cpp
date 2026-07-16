#include "c2_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <regex>

namespace {
std::optional<double> json_number(const std::string& json, const char* key) {
    const std::regex pattern(std::string("\\\"") + key + R"(\"\s*:\s*(-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?))");
    std::smatch match;
    if (!std::regex_search(json, match, pattern) || match.size() < 2) {
        return std::nullopt;
    }
    try {
        return std::stod(match[1].str());
    } catch (...) {
        return std::nullopt;
    }
}

std::string fmt_number(double value) {
    if (std::isfinite(value) && std::fabs(value - std::round(value)) < 1e-9) {
        return std::to_string(static_cast<long long>(std::llround(value)));
    }
    std::string s = std::to_string(value);
    while (!s.empty() && s.back() == '0') {
        s.pop_back();
    }
    if (!s.empty() && s.back() == '.') {
        s.pop_back();
    }
    return s;
}
}

C2Controller::C2Controller(FcLink& fc, UdpSocket& waypoints, std::ostream& log)
    : fc_(fc), waypoints_(waypoints), log_(log) {}

void C2Controller::tick() {
    fc_.poll();
    mark_healthy_once();
    update_state_from_fc();

    while (auto waypoint = waypoints_.receive_text()) {
        handle_waypoint(*waypoint);
    }
}

C2Controller::C2State C2Controller::current_state() const {
    return state_;
}

void C2Controller::transition(C2State next) {
    if (next == state_) {
        return;
    }

    const C2State prev = state_;
    state_ = next;
    log_ << "[C2] state: " << state_name(prev) << " -> " << state_name(next) << '\n';
    log_.flush();

    if (next == C2State::ARMED_HOLD) {
        fc_.hold();
    }
}

void C2Controller::update_state_from_fc() {
    if (!fc_.is_connected()) {
        transition(C2State::DISARMED);
        return;
    }

    if (!fc_.is_armed()) {
        transition(C2State::DISARMED);
        return;
    }

    const std::string mode = fc_.flight_mode();
    if (mode == "Guided") {
        transition(C2State::ARMED_GUIDED);
    } else if (mode == "Hold") {
        transition(C2State::ARMED_HOLD);
    } else if (mode == "Manual") {
        transition(C2State::ARMED_MANUAL);
    } else {
        transition(C2State::ARMED_MANUAL);
    }
}

void C2Controller::handle_waypoint(const std::string& json) {
    if (state_ != C2State::ARMED_GUIDED) {
        log_ << "[C2] blocked: waypoint in " << state_name(state_) << '\n';
        log_.flush();
        return;
    }

    const auto north = json_number(json, "north_m");
    const auto east = json_number(json, "east_m");
    if (!north || !east) {
        log_ << "[C2] blocked: invalid waypoint JSON\n";
        log_.flush();
        return;
    }

    fc_.go_to_ned(*north, *east);
    log_ << "[C2] fwd: north=" << fmt_number(*north) << " east=" << fmt_number(*east) << '\n';
    log_.flush();
}

void C2Controller::mark_healthy_once() {
    if (health_marked_ || !fc_.is_connected()) {
        return;
    }
    std::ofstream("/tmp/c2_healthy").close();
    health_marked_ = true;
}

const char* C2Controller::state_name(C2State state) {
    switch (state) {
        case C2State::DISARMED: return "DISARMED";
        case C2State::ARMED_HOLD: return "ARMED_HOLD";
        case C2State::ARMED_GUIDED: return "ARMED_GUIDED";
        case C2State::ARMED_MANUAL: return "ARMED_MANUAL";
    }
    return "UNKNOWN";
}
