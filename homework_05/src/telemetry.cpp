#include "telemetry.hpp"

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

constexpr int expected_field_count = 7;
constexpr int max_line_length = 256;
constexpr double low_voltage_threshold_v = 22.0;

std::runtime_error invalid_frame_error(int line_number, const std::string& reason) {
    std::ostringstream message;
    message << "invalid frame at line " << line_number << ": " << reason;
    return std::runtime_error{message.str()};
}

int split_line(char line[], char* fields[], int max_fields) {
    int count = 0;
    char* cursor = line;

    while (*cursor != '\0' && count < max_fields) {
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
            *cursor = '\0';
            ++cursor;
        }

        if (*cursor == '\0') {
            break;
        }

        fields[count] = cursor;
        ++count;

        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' &&
               *cursor != '\r') {
            ++cursor;
        }
    }

    return count;
}

long parse_long(const char* text, int line_number, const char* field_name) {
    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(text, &end, 10);

    if (end == text || *end != '\0' || errno == ERANGE) {
        throw invalid_frame_error(line_number, std::string{"invalid number in "} + field_name);
    }

    return value;
}

int parse_int(const char* text, int line_number, const char* field_name) {
    const long value = parse_long(text, line_number, field_name);
    if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
        throw invalid_frame_error(line_number, std::string{"integer out of range in "} + field_name);
    }

    return static_cast<int>(value);
}

double parse_double(const char* text, int line_number, const char* field_name) {
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(text, &end);

    if (end == text || *end != '\0' || errno == ERANGE) {
        throw invalid_frame_error(line_number, std::string{"invalid number in "} + field_name);
    }

    return value;
}

Frame parse_frame(char line[], int line_number) {
    char* fields[expected_field_count + 1] = {};
    const int field_count = split_line(line, fields, expected_field_count + 1);
    if (field_count != expected_field_count) {
        throw invalid_frame_error(line_number, "expected 7 fields");
    }

    Frame frame{};
    frame.timestamp_ms = parse_long(fields[0], line_number, "timestamp_ms");
    frame.seq = parse_int(fields[1], line_number, "seq");
    frame.voltage_v = parse_double(fields[2], line_number, "voltage_v");
    frame.current_a = parse_double(fields[3], line_number, "current_a");
    frame.temperature_c = parse_double(fields[4], line_number, "temperature_c");
    frame.gps_fix = parse_int(fields[5], line_number, "gps_fix");
    frame.satellites = parse_int(fields[6], line_number, "satellites");

    if (frame.voltage_v <= 0.0) {
        throw invalid_frame_error(line_number, "voltage_v must be positive");
    }

    if (frame.temperature_c < -40.0 || frame.temperature_c > 120.0) {
        throw invalid_frame_error(line_number, "temperature_c must be in range [-40, 120]");
    }

    if (frame.gps_fix != 0 && frame.gps_fix != 1) {
        throw invalid_frame_error(line_number, "gps_fix must be 0 or 1");
    }

    if (frame.satellites < 0) {
        throw invalid_frame_error(line_number, "satellites must be non-negative");
    }

    return frame;
}

double compute_frame_rate_hz(const Frame frames[], int frame_count) {
    if (frame_count < 2) {
        return 0.0;
    }

    const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;
    if (elapsed_ms <= 0) {
        throw std::runtime_error{"invalid telemetry log: elapsed time must be positive"};
    }

    return static_cast<double>(frame_count - 1) * 1000.0 / static_cast<double>(elapsed_ms);
}

}  // namespace

int read_frames(const char* path, Frame frames[], int max_frames) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error{std::string{"failed to open input file: "} + path};
    }

    int frame_count = 0;
    int line_number = 0;
    char line[max_line_length];

    while (input.getline(line, max_line_length)) {
        ++line_number;
        if (line[0] == '\0') {
            continue;
        }

        if (frame_count >= max_frames) {
            throw std::runtime_error{"too many telemetry frames"};
        }

        const Frame frame = parse_frame(line, line_number);
        if (frame_count > 0) {
            const Frame& previous = frames[frame_count - 1];
            if (frame.timestamp_ms <= previous.timestamp_ms) {
                throw invalid_frame_error(line_number, "timestamp_ms must increase");
            }

            if (frame.seq != previous.seq + 1) {
                throw invalid_frame_error(line_number, "seq must increase by 1");
            }
        }

        frames[frame_count] = frame;
        ++frame_count;
    }

    if (!input.eof()) {
        throw std::runtime_error{"failed to read input file"};
    }

    if (frame_count == 0) {
        throw std::runtime_error{"empty telemetry log"};
    }

    return frame_count;
}

Summary summarize(const Frame frames[], int frame_count) {
    Summary summary{};
    summary.frames_total = frame_count;
    summary.frames_valid = frame_count;
    summary.voltage_min = frames[0].voltage_v;
    summary.voltage_max = frames[0].voltage_v;
    summary.low_voltage_frames = 0;

    double temperature_sum = 0.0;

    for (int i = 0; i < frame_count; ++i) {
        if (frames[i].voltage_v < summary.voltage_min) {
            summary.voltage_min = frames[i].voltage_v;
        }

        if (frames[i].voltage_v > summary.voltage_max) {
            summary.voltage_max = frames[i].voltage_v;
        }

        temperature_sum += frames[i].temperature_c;

        if (frames[i].voltage_v < low_voltage_threshold_v) {
            ++summary.low_voltage_frames;
        }
    }

    summary.temperature_avg = temperature_sum / static_cast<double>(frame_count);
    summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
    return summary;
}

void print_summary(const Summary& summary) {
    std::cout << "frames_total " << summary.frames_total << '\n';
    std::cout << "frames_valid " << summary.frames_valid << '\n';
    std::cout << "voltage_min " << summary.voltage_min << '\n';
    std::cout << "voltage_max " << summary.voltage_max << '\n';
    std::cout << "temperature_avg " << summary.temperature_avg << '\n';
    std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
    std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
