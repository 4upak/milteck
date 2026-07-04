#include <exception>
#include <iostream>
#include <string>

#include "MissionProcessor.h"
#include "Types.h"

namespace {
void printUsage()
{
  std::cerr << "usage: homework_11_cli [uart_device] [gpio_chip] [start_line] [drop_line]\n";
}
}

int main(int argc, char** argv)
{
  homework_11::RuntimeConfig cfg;
  if (argc > 5) {
    printUsage();
    return 1;
  }
  if (argc >= 2) cfg.uart_device = argv[1];
  if (argc >= 3) cfg.gpio_chip = argv[2];
  if (argc >= 4) cfg.start_line = static_cast<unsigned>(std::stoul(argv[3]));
  if (argc >= 5) cfg.drop_line = static_cast<unsigned>(std::stoul(argv[4]));

  try {
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
