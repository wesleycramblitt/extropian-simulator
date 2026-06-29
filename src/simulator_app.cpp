// Simulator application implementation.
// Most logic is in main.cpp (simplex single-file app).
// This file exists for the CMake build target.
#include <cstdio>
namespace extropian_simulator {
void init() { std::printf("[ExtropianSimulator] Initialized.\n"); }
}
