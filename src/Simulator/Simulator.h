//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_SIMULATOR_H
#define FEM_SIMULATOR_H

#include "GUI/GUI.hpp"
#include "System.h"
#include "TimeStepper.h"

#include <string>

glm::vec3 Json2GlmVec3(const json& vec);

class Simulator : public GUI {
public:
    Simulator(const std::string& title) : GUI(title) {}

    void InitializeScene(Scene &scene) override;
    void Processing(Scene &scene) override;

    virtual void LoadScene(const std::string& config);
    void Simulate(const std::string& output_dir);

    ~Simulator() override;

protected:
    double _duration;
    double _time_step;
    System *_system;
    TimeStepper* _time_stepper;

    std::vector<int> _obj_id2scene_id; // from id in system to id in scene

    glm::vec3 _camera_position, _camera_look, _camera_up;
    glm::vec3 _light_position, _light_ambient, _light_diffuse, _light_specular;
    float _light_Kc, _light_Kl, _light_Kq;
};

#endif //FEM_SIMULATOR_H
