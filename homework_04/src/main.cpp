#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

// Параметри енкодерів і геометрії платформи.
constexpr long ticks_per_revolution{1024};
constexpr double wheel_radius_m{0.3};
constexpr double wheelbase_m{1.0};
constexpr double pi{3.14159265358979323846};
// Відстань, яку колесо проходить за один імпульс енкодера.
constexpr double distance_per_tick{
    2.0 * pi * wheel_radius_m / static_cast<double>(ticks_per_revolution)};

struct EncoderSample {
    // Час вимірювання в мілісекундах.
    long timestamp_ms{};
    // Значення лічильників для переднього лівого, переднього правого,
    // заднього лівого та заднього правого коліс.
    long fl_ticks{};
    long fr_ticks{};
    long bl_ticks{};
    long br_ticks{};
};

struct Pose {
    // Поточна оцінка положення робота на площині та його орієнтації.
    double x{};
    double y{};
    double theta{};
};

enum class ReadSampleStatus {
    ok,
    eof,
    error,
};

bool is_blank_line(const std::string& line)
{
    for (const unsigned char ch : line) {
        if (!std::isspace(ch)) {
            return false;
        }
    }

    return true;
}

std::string build_open_error_message(const std::string& input_path)
{
    const std::filesystem::path path{input_path};
    std::error_code error_code{};

    if (!std::filesystem::exists(path, error_code)) {
        if (error_code) {
            return "Error: cannot access input file '" + input_path +
                   "': " + error_code.message();
        }

        return "Error: input file does not exist: " + input_path;
    }

    if (!std::filesystem::is_regular_file(path, error_code)) {
        if (error_code) {
            return "Error: cannot inspect input file '" + input_path +
                   "': " + error_code.message();
        }

        return "Error: input path is not a regular file: " + input_path;
    }

    return "Error: cannot open input file: " + input_path;
}

ReadSampleStatus read_sample(std::istream& input, EncoderSample& sample,
                             std::size_t& line_number, std::string& error_message)
{
    std::string line{};

    while (std::getline(input, line)) {
        ++line_number;

        if (is_blank_line(line)) {
            continue;
        }

        std::istringstream line_stream{line};
        if (!(line_stream >> sample.timestamp_ms >> sample.fl_ticks
              >> sample.fr_ticks >> sample.bl_ticks >> sample.br_ticks)) {
            error_message = "Error: invalid data on line " +
                            std::to_string(line_number) +
                            ": expected 5 integer values";
            return ReadSampleStatus::error;
        }

        line_stream >> std::ws;
        if (!line_stream.eof()) {
            std::string extra_data{};
            line_stream >> extra_data;
            error_message = "Error: unexpected extra data on line " +
                            std::to_string(line_number);
            if (!extra_data.empty()) {
                error_message += ": '" + extra_data + "'";
            }
            return ReadSampleStatus::error;
        }

        return ReadSampleStatus::ok;
    }

    if (input.bad()) {
        error_message = "Error: I/O failure while reading the input file";
        return ReadSampleStatus::error;
    }

    return ReadSampleStatus::eof;
}

void update_pose(Pose& pose, const EncoderSample& previous,
                 const EncoderSample& current)
{
    // Обчислюємо приріст імпульсів для кожного колеса між двома вимірюваннями.
    const long d_fl{current.fl_ticks - previous.fl_ticks};
    const long d_fr{current.fr_ticks - previous.fr_ticks};
    const long d_bl{current.bl_ticks - previous.bl_ticks};
    const long d_br{current.br_ticks - previous.br_ticks};

    // Усереднюємо ліву та праву сторони, щоб отримати зсув платформи.
    const double d_left_ticks{(static_cast<double>(d_fl) + d_bl) / 2.0};
    const double d_right_ticks{(static_cast<double>(d_fr) + d_br) / 2.0};

    const double d_left_m{d_left_ticks * distance_per_tick};
    const double d_right_m{d_right_ticks * distance_per_tick};

    // Модель диференціального приводу: лінійне зміщення та зміна кута.
    const double distance_m{(d_left_m + d_right_m) / 2.0};
    const double dtheta{(d_right_m - d_left_m) / wheelbase_m};
    const double middle_theta{pose.theta + dtheta / 2.0};

    // Оновлюємо позу, використовуючи орієнтацію в середині кроку.
    pose.x += distance_m * std::cos(middle_theta);
    pose.y += distance_m * std::sin(middle_theta);
    pose.theta += dtheta;
}

}  // namespace

int main(int argc, char* argv[])
{
    // Очікуємо шлях до вхідного файлу як єдиний аргумент командного рядка.
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file>\n";
        return 1;
    }

    const std::string input_path{argv[1]};
    std::ifstream input{input_path};
    // Перевіряємо, що файл успішно відкрився.
    if (!input) {
        std::cerr << build_open_error_message(input_path) << '\n';
        return 1;
    }

    EncoderSample previous{};
    std::size_t line_number{};
    std::string read_error{};
    // Перше вимірювання потрібне як базова точка для подальших приростів.
    const ReadSampleStatus first_read_status{
        read_sample(input, previous, line_number, read_error)};
    if (first_read_status == ReadSampleStatus::eof) {
        std::cerr << "Error: input file is empty or contains only blank lines\n";
        return 1;
    }

    if (first_read_status == ReadSampleStatus::error) {
        std::cerr << read_error << '\n';
        return 1;
    }

    Pose pose{};
    EncoderSample current{};
    // Для кожного нового вимірювання оновлюємо позу та виводимо результат.
    while (true) {
        const ReadSampleStatus status{
            read_sample(input, current, line_number, read_error)};
        if (status == ReadSampleStatus::eof) {
            break;
        }

        if (status == ReadSampleStatus::error) {
            std::cerr << read_error << '\n';
            return 1;
        }

        update_pose(pose, previous, current);
        std::cout << current.timestamp_ms << ' ' << pose.x << ' ' << pose.y
                  << ' ' << pose.theta << '\n';
        previous = current;
    }

    return 0;
}
