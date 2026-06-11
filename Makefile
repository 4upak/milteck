.PHONY: format lint test quality

HOMEWORK_06_CPP := \
	homework_06/include/ballistics.hpp \
	homework_06/src/ballistics.cpp \
	homework_06/src/main.cpp \
	homework_06/tests/ballistics_tests.cpp

HOMEWORK_07_HPP := \
	homework_07/include/types.hpp \
	homework_07/include/i_target_provider.hpp \
	homework_07/include/i_ballistic_solver.hpp \
	homework_07/include/i_config_loader.hpp \
	homework_07/include/json_target_provider.hpp \
	homework_07/include/analytical_solver.hpp \
	homework_07/include/file_config_loader.hpp \
	homework_07/include/factory.hpp \
	homework_07/include/mission_processor.hpp

HOMEWORK_07_CPP := \
	homework_07/src/analytical_solver.cpp \
	homework_07/src/json_target_provider.cpp \
	homework_07/src/file_config_loader.cpp \
	homework_07/src/factory.cpp \
	homework_07/src/mission_processor.cpp \
	homework_07/src/main.cpp \
	homework_07/tests/homework_07_tests.cpp

HOMEWORK_08_H := \
	homework_08/include/Types.h \
	homework_08/include/MissionProcessor.h \
	homework_08/include/interfaces/ITargetProvider.h \
	homework_08/include/interfaces/IBallisticSolver.h \
	homework_08/include/interfaces/IConfigLoader.h \
	homework_08/include/solvers/AnalyticalSolver.h \
	homework_08/include/providers/JsonTargetProvider.h \
	homework_08/include/config/FileConfigLoader.h \
	homework_08/include/config/ComponentFactory.h

HOMEWORK_08_CPP := \
	homework_08/src/MissionProcessor.cpp \
	homework_08/src/solvers/AnalyticalSolver.cpp \
	homework_08/src/providers/JsonTargetProvider.cpp \
	homework_08/src/config/FileConfigLoader.cpp \
	homework_08/src/config/ComponentFactory.cpp \
	homework_08/src/main.cpp \
	homework_08/tests/homework_08_tests.cpp

format:
	clang-format --style=file:.devcontainer/.clang-format -i $(HOMEWORK_06_CPP) $(HOMEWORK_07_HPP) $(HOMEWORK_07_CPP) $(HOMEWORK_08_H) $(HOMEWORK_08_CPP)
	cmake-format -c .devcontainer/.cmake-format.json -i homework_06/CMakeLists.txt homework_07/CMakeLists.txt homework_08/CMakeLists.txt

lint:
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_06/src/ballistics.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_06/src/main.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_06/tests/ballistics_tests.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_07/src/analytical_solver.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_07/src/json_target_provider.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_07/src/file_config_loader.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_07/src/factory.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_07/src/mission_processor.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_07/src/main.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_07/tests/homework_07_tests.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_08/src/MissionProcessor.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_08/src/solvers/AnalyticalSolver.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_08/src/providers/JsonTargetProvider.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_08/src/config/FileConfigLoader.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_08/src/config/ComponentFactory.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_08/src/main.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_08/tests/homework_08_tests.cpp

test:
	cmake --preset debug
	cmake --build --preset debug
	ctest --test-dir build/debug --output-on-failure

quality: format test lint
