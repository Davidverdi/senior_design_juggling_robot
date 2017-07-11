# makefile
# Created 4/16/2017
# David Verdi

# This is the makefile that compiles the motor control code

# First, set the default compiler
CXX = g++
CC = g++

# Compilation options/tags
CXXFLAGS = -g -Wall -pthread $(INCLUDES)

# Linking Options
LDFLAGS = 

# Libraries
LDLIBS = -lpigpio -lrt -lm

# The following is the first target. 
# This is the one that gets executed by default if you just type in make into 
# the terminal
# make automatically does $(CXX) $(LDFLAGS) <all-dependant-.o-files> $(LDLIBS)
main: main.o dc_motor.o rot_encoder.o motor_sync.o udp_connection.o

# The following are the object file dependencies. 
# make automatically does $(CXX) -c $(CFLAGS) <cpp-files>
main.o: main.cpp main.hpp
dc_motor.o: dc_motor.cpp dc_motor.hpp 
rot_encoder.o: rot_encoder.cpp rot_encoder.hpp
motor_sync.o: motor_sync.cpp motor_sync.hpp
udp_connection.o: udp_connection.cpp udp_connection.hpp

# The clean target will do the function of cleaning out the intermediaries when
# run as make clean
# The clean target is not a filename, so we indicate this to make by adding 
# in .PHONY:
# This tells make that clean is a phony target
.PHONY: clean
clean:
	rm -f *.o a.out core main

# The all target will clean, then rebuild the main target
.PHONY: all
all: clean main
