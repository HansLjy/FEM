//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_SIMULATOR_H
#define FEM_SIMULATOR_H

#include "GUI/GUI.hpp"
#include "System.h"
#include "Integrator/FastProjection.h"

#include <string>

class Simulator : public GUI {
public:
    void InitializeScene(Scene &scene) override;
    void Processing(Scene &scene) override;

    void LoadScene(const std::string& config);
    void Simulate();

    ~Simulator();

private:
    double _duration;
    double _time_step;
    System _system;
    const Integrator* _integrator;

    std::vector<int> _obj_scene_id; // from id in system to id in scene
};

#endif //FEM_SIMULATOR_H
