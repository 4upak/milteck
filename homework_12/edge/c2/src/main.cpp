#include "c2_controller.hpp"
#include "fc_link.hpp"
#include "udp_socket.hpp"

#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t g_running = 1;

void handle_signal(int) {
    g_running = 0;
}

int read_int_config(const std::string& text, const char* key, int fallback) {
    const std::regex pattern(std::string("\\\"") + key + R"(\"\s*:\s*(\d+))");
    std::smatch match;
    if (!std::regex_search(text, match, pattern) || match.size() < 2) {
        return fallback;
    }
    try {
        return std::stoi(match[1].str());
    } catch (...) {
        return fallback;
    }
}

std::string read_string_config(const std::string& text, const char* key, const std::string& fallback) {
    const std::regex pattern(std::string("\\\"") + key + R"(\"\s*:\s*\"([^\"]+)\")");
    std::smatch match;
    if (!std::regex_search(text, match, pattern) || match.size() < 2) {
        return fallback;
    }
    return match[1].str();
}
}

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const std::string config_path = "/etc/c2/c2_config.json";
    std::ifstream cfg(config_path);
    if (!cfg) {
        std::cerr << "[C2] failed to open config: " << config_path << '\n';
        return 1;
    }
    const std::string config_text((std::istreambuf_iterator<char>(cfg)), std::istreambuf_iterator<char>());

    const int fc_port = read_int_config(config_text, "fc_port", 14551);
    const int waypoint_port = read_int_config(config_text, "waypoint_port", 14560);
    const std::string log_file = read_string_config(config_text, "log_file", "/var/log/c2/c2.log");

    std::filesystem::create_directories(std::filesystem::path(log_file).parent_path());
    std::ofstream log(log_file, std::ios::app);
    if (!log) {
        std::cerr << "[C2] failed to open log file: " << log_file << '\n';
        return 1;
    }

    log << "[C2] config: fc_port=" << fc_port << '\n';
    log.flush();
    std::cout << "[C2] config: fc_port=" << fc_port << std::endl;

    try {
        FcLink fc(static_cast<uint16_t>(fc_port));
        UdpSocket waypoint_socket(static_cast<uint16_t>(waypoint_port));
        C2Controller controller(fc, waypoint_socket, log);

        while (g_running) {
            controller.tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } catch (const std::exception& exc) {
        log << "[C2] fatal: " << exc.what() << '\n';
        log.flush();
        std::cerr << "[C2] fatal: " << exc.what() << '\n';
        return 1;
    }

    return 0;
}
