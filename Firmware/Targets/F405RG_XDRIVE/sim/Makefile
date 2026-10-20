# Host simulation of MotorXDrive (Linux / WSL, g++ >= 10)
#   make run
FFB = ../../../FFBoard
CXXFLAGS = -std=gnu++20 -O2 -Wall -Wno-unused-variable -Wno-volatile -g \
	-Istubs -I$(FFB)/UserExtensions/Inc

sim: sim.cpp $(FFB)/UserExtensions/Src/MotorXDrive.cpp $(FFB)/UserExtensions/Inc/MotorXDrive.h stubs/*.h
	$(CXX) $(CXXFLAGS) sim.cpp $(FFB)/UserExtensions/Src/MotorXDrive.cpp -o sim

run: sim
	./sim

clean:
	rm -f sim
