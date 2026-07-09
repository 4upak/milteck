#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "MissionProcessor.h"
#include "Types.h"

namespace {

void printUsage()
{
  std::cerr << "usage: homework_11_cli [--uart <dev>] [--gpiochip <name>] "
               "[--start-line <n>] [--drop-line <n>]\n"
               "   or: homework_11_cli [uart_device] [gpio_chip] [start_line] [drop_line]\n";
}

[[nodiscard]] unsigned parseLine(std::string_view value)
{
  return static_cast<unsigned>(std::stoul(std::string(value)));
}

// Parse both the assignment's flag style (--uart /tmp/ttyA --gpiochip ...)
// and the legacy positional style. Returns false on malformed input.
[[nodiscard]] bool parseArgs(int argc, char** argv, homework_11::RuntimeConfig& cfg)
{
  std::vector<std::string_view> positional;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    const auto needValue = [&](std::string_view name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "error: missing value for " << name << '\n';
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "-h" || arg == "--help") {
      printUsage();
      return false;
    }
    if (arg == "--uart") {
      const char* value = needValue(arg);
      if (!value) return false;
      cfg.uart_device = value;
    }
    else if (arg == "--gpiochip") {
      const char* value = needValue(arg);
      if (!value) return false;
      cfg.gpio_chip = value;
    }
    else if (arg == "--start-line") {
      const char* value = needValue(arg);
      if (!value) return false;
      cfg.start_line = parseLine(value);
    }
    else if (arg == "--drop-line") {
      const char* value = needValue(arg);
      if (!value) return false;
      cfg.drop_line = parseLine(value);
    }
    else if (!arg.empty() && arg.front() == '-') {
      std::cerr << "error: unknown option " << arg << '\n';
      return false;
    }
    else {
      positional.push_back(arg);
    }
  }

  if (positional.size() > 4) {
    std::cerr << "error: too many positional arguments\n";
    return false;
  }
  if (positional.size() >= 1) cfg.uart_device = std::string(positional[0]);
  if (positional.size() >= 2) cfg.gpio_chip = std::string(positional[1]);
  if (positional.size() >= 3) cfg.start_line = parseLine(positional[2]);
  if (positional.size() >= 4) cfg.drop_line = parseLine(positional[3]);
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  homework_11::RuntimeConfig cfg;

  try {
    if (!parseArgs(argc, argv, cfg)) {
      return 1;
    }

    homework_11::MissionProcessor::installSignalHandlers();
    homework_11::MissionProcessor mission(cfg);
    mission.init();
    mission.run();
  }
  catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
