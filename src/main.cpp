//
// Created by hansljy on 10/13/22.
//

#include "Simulator.h"

int main() {
    Simulator simulator("FEM Simulator");
    simulator.LoadScene(CONFIG_PATH "/test.json");
    simulator.MainLoop();
//    simulator.Simulate();
}