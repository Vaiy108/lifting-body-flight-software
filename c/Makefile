# Dependency-free build: only libc and libm. No external libraries.
CC ?= gcc
CFLAGS = -std=c11 -Wall -Wextra -O2 -Iinclude
LDFLAGS = -lm

SRC = src/quat_math.c src/matlib.c src/eskf.c src/control.c src/pil_protocol.c
OBJ = $(SRC:src/%.c=build/%.o)

.PHONY: all test test-pil clean

all: build/test_main build/host_sim

build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/pil_core.o: pil/pil_core.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/test_main: test/test_main.c $(OBJ) | build
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

build/host_sim: pil/host_sim_main.c $(OBJ) build/pil_core.o | build
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

build:
	mkdir -p build

test: build/test_main
	./build/test_main test_vectors

test-pil: build/host_sim
	@echo "host_sim built. Run the PC-side driver against it with:"
	@echo "  cd ../python && python3 scripts/pil_driver.py --backend hostsim"

clean:
	rm -rf build
