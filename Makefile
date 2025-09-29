# --- config ---
CXX := g++
CXXFLAGS := -O2 -std=c++17 -D__LINUX__ -Wall -Wextra
PIXY2_ROOT := $(HOME)/pixy2

INCLUDES := \
  -I$(PIXY2_ROOT)/src/host/libpixyusb2/include \
  -I$(PIXY2_ROOT)/src/host/libpixyusb2/inc \
  -I$(PIXY2_ROOT)/src/common/inc \
  -I$(PIXY2_ROOT)/src/host/arduino/libraries/Pixy2

LDFLAGS := -L$(PIXY2_ROOT)/build/libpixyusb2 -Wl,-rpath,$(PIXY2_ROOT)/build/libpixyusb2
LDLIBS := -lpixy2 -pthread $(shell pkg-config --libs libusb-1.0)
CXXFLAGS += $(shell pkg-config --cflags libusb-1.0)

SRC := line_follow.cpp uart.cpp
OBJ := $(SRC:.cpp=.o)
BIN := line_follow

all: $(BIN)

$(BIN): $(OBJ)
	$(CXX) $(OBJ) $(LDFLAGS) $(LDLIBS) -o $@

%.o: %.cpp uart.hpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJ) $(BIN)

run: $(BIN)
	./$(BIN)

# Build Pixy2 host lib if it's missing (top-level CMake build)
.PHONY: pixy-build
pixy-build:
	@if [ ! -f "$(PIXY2_ROOT)/build/libpixyusb2/libpixy2.so" ]; then \
	  echo "[pixy-build] lib not found, configuring & building pixy2 top-level..."; \
	  mkdir -p "$(PIXY2_ROOT)/build"; \
	  cd "$(PIXY2_ROOT)/build" && cmake .. && $(MAKE) -j$$(nproc); \
	else \
	  echo "[pixy-build] lib already present: $(PIXY2_ROOT)/build/libpixyusb2/libpixy2.so"; \
	fi

# Optional local overrides
-include Makefile.local
