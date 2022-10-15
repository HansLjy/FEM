//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_SIMULATOR_H
#define FEM_SIMULATOR_H

#include "GUI/GUI.hpp"
#include "System.h"
#include "Integrator/FastProjection.h"

#include <string>

glm::vec3 Json2GlmVec3(const json& vec);

class Simulator : public GUI {
public:
    Simulator(const std::string& title) : GUI(title) {}

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

    glm::vec3 _camera_position, _camera_look, _camera_up;
    glm::vec3 _light_position, _light_ambient, _light_diffuse, _light_specular;
    float _light_Kc, _light_Kl, _light_Kq;
};

#endif //FEM_SIMULATOR_H