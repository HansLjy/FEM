//
// Created by hansljy on 10/11/22.
//

#include "Simulator.h"
#include "Curve/Curve.h"
#include "Integrator/Integrator.h"
#include "Shape.h"
#include "nlohmann/json.hpp"
#include <fstream>

using nlohmann::json;

glm::vec3 Json2GlmVec3(const json& vec) {
    return glm::vec3(vec[0], vec[1], vec[2]);
}

void Simulator::LoadScene(const std::string &config) {
    json config_json;
    std::fstream config_file(config);
    config_file >> config_json;
    auto simulation_config = config_json["simulation-config"];
    _duration = simulation_config["duration"];
    _time_step = simulation_config["time-step"];

    const auto& integrator_config = config_json["integrator"];
    _integrator = IntegratorFactory::GetIntegrator(integrator_config["type"], integrator_config);

    const auto& objects_config = config_json["objects"];
    for (const auto& object_config : objects_config) {
        _system.AddObject(*ObjectFactory::GetObject(object_config["type"], object_config), object_config["name"]);
    }

    const auto& constraints_config = config_json["constraints"];
    for (const auto& constraint_config : constraints_config) {
        _system.AddConstraint(*ConstraintFactory::GetConstraint(_system, constraint_config));
    }

    const auto& external_forces_config = config_json["external-forces"];
    for (const auto& external_force_config : external_forces_config) {
        int idx = _system.GetIndex(external_force_config["object-name"]);
        const auto& external_force = ExternalForceFactory::GetExternalForce(external_force_config["type"], external_force_config);
        _system.GetObject(idx)->AddExternalForce(*external_force);
    }

    _system.UpdateSettings();

    const auto& renderer_config_json = config_json["renderer"];
    const auto& camera_config_json = renderer_config_json["camera"];
    _camera_position = Json2GlmVec3(camera_config_json["position"]);
    _camera_look = Json2GlmVec3(camera_config_json["look"]);
    _camera_up = Json2GlmVec3(camera_config_json["up"]);

    const auto& light_config_json = renderer_config_json["light"];
    _light_position = Json2GlmVec3(light_config_json["position"]);
    _light_ambient = Json2GlmVec3(light_config_json["ambient"]);
    _light_diffuse = Json2GlmVec3(light_config_json["diffuse"]);
    _light_specular = Json2GlmVec3(light_config_json["specular"]);
    _light_Kc = light_config_json["Kc"];
    _light_Kl = light_config_json["Kl"];
    _light_Kq = light_config_json["Kq"];

}

void Simulator::Simulate() {
    double current_time = 0;
    while(current_time < _duration) {
        VectorXd x_next, v_next;
        _integrator->Step(_system, _time_step, x_next, v_next);
        _system.SetCoordinate(x_next);
        _system.SetVelocity(v_next);

        current_time += _time_step;
    }
}

Simulator::~Simulator() {
    delete _integrator;
}

void Simulator::InitializeScene(Scene &scene) {
    SetCamera(
        _camera_position,
        _camera_look,
        _camera_up
    );
    SetLight(
        _light_position,
        _light_ambient,
        _light_diffuse,
        _light_specular,
        _light_Kc, _light_Kl, _light_Kq
    );

    const auto& objs = _system._objs;
    _obj_scene_id.resize(_system._size);
    for (int i = 0; i < _system._size; i++) {
        if (_system._used[i]) {
            const auto& obj = objs[i];
            const auto& shape = obj->GetShape();
            MatrixXd vertices;
            MatrixXi topo;
            shape->GetSurface(*obj, vertices, topo);
            _obj_scene_id[i] = scene.AddMesh(vertices, topo);
        }
    }
}

void Simulator::Processing(Scene &scene) {
    VectorXd x_next, v_next;
    _integrator->Step(_system, _time_step, x_next, v_next);
    _system.SetCoordinate(x_next);
    _system.SetVelocity(v_next);
    const auto& objs = _system._objs;
    for (int i = 0; i < _system._size; i++) {
        if (_system._used[i]) {
            const auto& obj = objs[i];
            const auto& shape = obj->GetShape();
            MatrixXd vertices;
            MatrixXi topo;
            shape->GetSurface(*obj, vertices, topo);
            scene.SelectData(_obj_scene_id[i]);
            scene.SetMesh(vertices, topo);
        }
    }
}