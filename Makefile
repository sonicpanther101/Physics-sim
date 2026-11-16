CC = g++
CFLAGS = $(shell pkg-config --cflags sdl3 glm)
# CFLAGS = -Wall -Werror $(shell pkg-config --cflags sdl3)
LDFLAGS = $(shell pkg-config --libs sdl3 glm)
DEL = rm -f

all: sim.out

sim.out: src/main.cpp src/physics.cpp
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

.PHONY: clean
clean:
	$(DEL) *.o *.out

.PHONY: program
program: sim.out
	./sim.out
