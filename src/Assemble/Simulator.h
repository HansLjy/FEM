#pragma once

#include "Pattern.h"
#include "GUI/GUI.hpp"
#include "TimeStepper/TimeStepper.hpp"
#include <string>
#include "Render/RenderInterface.hpp"

glm::vec3 Json2GlmVec3(const json& vec);

class Simulator : public GUI {
public:
    Simulator(bool display_bb, const std::string& title) : GUI(display_bb, title) {}

    void InitializeScene(Scene &scene) override;
    void Processing(Scene &scene) override;

    void LoadScene(const std::string& filename);
    void Simulate(const std::string& output_dir);

    ~Simulator() override;

protected:
    double _duration;
    double _time_step;
    TimeStepper* _time_stepper;

    std::vector<int> _obj_id2scene_id; // from id in system to id in scene

    glm::vec3 _camera_position, _camera_look, _camera_up;
    glm::vec3 _light_position, _light_ambient, _light_diffuse, _light_specular;
    float _light_Kc, _light_Kl, _light_Kq;
};

template<class T>
bool RegisterForRenderer(const std::string &type) {
    return CasterRegistration::RegisterForCaster<Renderable, T>(type);
}