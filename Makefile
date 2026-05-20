.PHONY: format lint test quality

HOMEWORK_06_CPP := \
	homework_06/include/ballistics.hpp \
	homework_06/src/ballistics.cpp \
	homework_06/src/main.cpp \
	homework_06/tests/ballistics_tests.cpp

format:
	clang-format --style=file:.devcontainer/.clang-format -i $(HOMEWORK_06_CPP)
	cmake-format -c .devcontainer/.cmake-format.json -i homework_06/CMakeLists.txt

lint:
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_06/src/ballistics.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_06/src/main.cpp
	clang-tidy --config-file=.devcontainer/.clang-tidy -p build/debug homework_06/tests/ballistics_tests.cpp

test:
	cmake --preset debug
	cmake --build --preset debug
	ctest --test-dir build/debug --output-on-failure

quality: format test lint
