#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

namespace {

constexpr long ticks_per_revolution{1024};
constexpr double wheel_radius_m{0.3};
constexpr double wheelbase_m{1.0};
constexpr double pi{3.14159265358979323846};
constexpr double distance_per_tick{
    2.0 * pi * wheel_radius_m / static_cast<double>(ticks_per_revolution)};

struct EncoderSample {
    long timestamp_ms{};
    long fl_ticks{};
    long fr_ticks{};
    long bl_ticks{};
    long br_ticks{};
};

struct Pose {
    double x{};
    double y{};
    double theta{};
};

bool read_sample(std::istream& input, EncoderSample& sample)
{
    return static_cast<bool>(input >> sample.timestamp_ms >> sample.fl_ticks
                             >> sample.fr_ticks >> sample.bl_ticks
                             >> sample.br_ticks);
}

void update_pose(Pose& pose, const EncoderSample& previous,
                 const EncoderSample& current)
{
    const long d_fl{current.fl_ticks - previous.fl_ticks};
    const long d_fr{current.fr_ticks - previous.fr_ticks};
    const long d_bl{current.bl_ticks - previous.bl_ticks};
    const long d_br{current.br_ticks - previous.br_ticks};

    const double d_left_ticks{(static_cast<double>(d_fl) + d_bl) / 2.0};
    const double d_right_ticks{(static_cast<double>(d_fr) + d_br) / 2.0};

    const double d_left_m{d_left_ticks * distance_per_tick};
    const double d_right_m{d_right_ticks * distance_per_tick};

    const double distance_m{(d_left_m + d_right_m) / 2.0};
    const double dtheta{(d_right_m - d_left_m) / wheelbase_m};
    const double middle_theta{pose.theta + dtheta / 2.0};

    pose.x += distance_m * std::cos(middle_theta);
    pose.y += distance_m * std::sin(middle_theta);
    pose.theta += dtheta;
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file>\n";
        return 1;
    }

    const std::string input_path{argv[1]};
    std::ifstream input{input_path};
    if (!input) {
        std::cerr << "Error: cannot open input file: " << input_path << '\n';
        return 1;
    }

    EncoderSample previous{};
    if (!read_sample(input, previous)) {
        std::cerr << "Error: input file is empty or has invalid format\n";
        return 1;
    }

    Pose pose{};
    EncoderSample current{};
    while (read_sample(input, current)) {
        update_pose(pose, previous, current);
        std::cout << current.timestamp_ms << ' ' << pose.x << ' ' << pose.y
                  << ' ' << pose.theta << '\n';
        previous = current;
    }

    if (!input.eof()) {
        std::cerr << "Error: invalid input format\n";
        return 1;
    }

    return 0;
}
