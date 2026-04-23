CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2
TARGET   := drone_sim
SRC      := main.cpp

.PHONY: all run clean rebuild

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) simulation.json

rebuild: clean all
