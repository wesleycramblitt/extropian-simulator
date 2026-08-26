// Extropian Simulator — main entry point (minimal).
//
// The full application lives in SimulatorApp (simulator_app.{hpp,cpp});
// this file only boots it.
#include <exd/app/application.hpp>

#include "simulator_app.hpp"

int main(int, char**) {
    SimulatorApp app;
    return app.run();
}
