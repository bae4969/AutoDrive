CXX = g++
CXXFLAGS = -std=c++20 -Wall -O2
TARGET = AutoDrive
SRCS = $(wildcard  src/Modules/*.cpp src/AutoDrive/*.cpp)
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)