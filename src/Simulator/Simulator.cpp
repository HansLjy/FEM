//
// Created by hansljy on 10/11/22.
//

#include "Simulator.h"
#include "Integrator/Integrator.h"
#include "Shape.h"
#include "nlohmann/json.hpp"
#include "JsonUtil.h"
#include <fstream>
#include <string>

using nlohmann::json;

void Simulator::LoadScene(const std::string &config) {
    json config_json;
    std::fstream config_file(config);
    config_file >> config_json;
    auto simulation_config = config_json["simulation-config"];
    _duration = simulation_config["duration"];
    _time_step = simulation_config["time-step"];

    const auto& integrator_config = config_json["integrator"];
    _integrator = IntegratorFactory::GetIntegrator(integrator_config["type"], integrator_config);

    const std::string system_config_file_path = config_json["system-config"];
    std::ifstream system_config_file(CONFIG_PATH + system_config_file_path);
    const json system_config = json::parse(system_config_file);
    system_config_file.close();
    _system = PhysicsSystemFactory::GetPhysicsSystem(system_config["type"], system_config);
    _system->UpdateSettings(system_config);

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

void Simulator::Simulate(const std::string& output_dir) {
    double current_time = 0;
    int itr_id = 0;
    int obj_id = 0;
	std::ofstream topo_file(output_dir + "/topo");
    for (auto itr = _system->GetIterator(); !itr->IsDone(); itr->Forward(), obj_id++) {
        const auto& obj = itr->GetObject();
        MatrixXi topo;
        MatrixXd vertices;
        obj->GetShape(vertices, topo);
        write_binary(topo_file, topo);
    }
	topo_file.close();
    const int object_number = obj_id;

    while(current_time < _duration) {
        _integrator->Step(*_system, _time_step);
        obj_id = 0;
		std::ofstream itr_file(output_dir + "/itr" + std::to_string(itr_id));
        for (auto itr = _system->GetIterator(); !itr->IsDone(); itr->Forward(), obj_id++) {
            const auto& obj = itr->GetObject();
            MatrixXi topo;
            MatrixXd vertices;
            obj->GetShape(vertices, topo);
            write_binary(itr_file, vertices);
            write_binary(itr_file, itr->GetRotation());
            write_binary(itr_file, itr->GetTranslation());
        }
		itr_file.close();
        current_time += _time_step;
        itr_id++;
        spdlog::info("Current time: {}", current_time);
    }
    json config;
    config["number-of-objects"] = object_number;
    config["number-of-iterations"] = itr_id;
    std::ofstream config_file(output_dir + "/" + "config.json");
    config_file << config.dump();
    config_file.close();
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

    int id = 0;
    for (auto itr = _system->GetIterator(); !itr->IsDone(); itr->Forward(), id++) {
        const auto obj = itr->GetObject();
        MatrixXd vertices;
        MatrixXi topo;
        obj->GetShape(vertices, topo);
//        std::cerr << "Rotation " << id << ":\n" << itr->GetRotation() << std::endl;
//        std::cerr << "Translation " << id << ":\n" << itr->GetTranslation().transpose() << std::endl;
        _obj_id2scene_id.push_back(scene.AddMesh(vertices, topo, itr->GetRotation(), itr->GetTranslation()));
    }
}

#include "Timer.h"

void Simulator::Processing(Scene &scene) {
    auto t = clock();
    _integrator->Step(*_system, _time_step);
    int id = 0;
    for (auto itr = _system->GetIterator(); !itr->IsDone(); itr->Forward(), id++) {
        const auto obj = itr->GetObject();
        MatrixXd vertices;
        MatrixXi topo;
        obj->GetShape(vertices, topo);
        scene.SelectData(_obj_id2scene_id[id]);
        scene.SetMesh(vertices, topo, itr->GetRotation(), itr->GetTranslation());
    }
    auto delta_t = clock() - t;
    spdlog::info("fps = {}", (int)(CLOCKS_PER_SEC / delta_t));
}