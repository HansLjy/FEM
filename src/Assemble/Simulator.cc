//
// Created by hansljy on 10/11/22.
//

#include "Simulator.h"
#include "Integrator/Integrator.h"
#include "nlohmann/json.hpp"
#include "JsonUtil.h"
#include <string>

using nlohmann::json;

void Simulator::LoadScene(const std::string &config) {
    json config_json;
    std::fstream config_file(config);
    config_file >> config_json;
    auto simulation_config = config_json["simulation-config"];
    _duration = simulation_config["duration"];
    _time_step = simulation_config["time-step"];

    const auto& time_stepper_config = config_json["time-stepper"];
    _time_stepper = Factory<TimeStepper>::GetInstance()->GetProduct(time_stepper_config["type"], time_stepper_config);

    const std::string system_config_file_path = config_json["system-config"];
    std::ifstream system_config_file(CONFIG_PATH + system_config_file_path);
    const json system_config = json::parse(system_config_file);
    system_config_file.close();
    _system = new System(system_config);
    _system->Initialize(system_config);

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

    _time_stepper->Bind(_system);

    spdlog::info("Scene loaded");
}

void Simulator::Simulate(const std::string& output_dir) {
    double current_time = 0;
    int itr_id = 0;
    int obj_id = 0;
	std::ofstream topo_file(output_dir + "/topo");

	for (const auto& obj : _system->_objs) {
        MatrixXi topo;
        MatrixXd vertices;
		obj->GetRenderTopos(topo);
		obj->GetRenderVertices(vertices);
        write_binary(topo_file, topo);
	}

	topo_file.close();
    const int object_number = obj_id;

    while(current_time < _duration) {
        _time_stepper->Step(_time_step);
        obj_id = 0;
		std::ofstream itr_file(output_dir + "/itr" + std::to_string(itr_id));
        for (const auto& obj : _system->_objs) {
            MatrixXd vertices;
			obj->GetRenderVertices(vertices);
            write_binary(itr_file, vertices);
            write_binary(itr_file, obj->GetFrameRotation());
            write_binary(itr_file, obj->GetFrameX());
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
    delete _time_stepper;
    delete _system;
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

    for (const auto& obj : _system->_objs) {
        MatrixXd vertices;
        MatrixXi topo;

		obj->GetRenderTopos(topo);
		obj->GetRenderVertices(vertices);

        _obj_id2scene_id.push_back(scene.AddMesh(vertices, topo, obj->GetFrameRotation(), obj->GetFrameX()));
        if (obj->IsUsingTexture()) {
            MatrixXf uv_coords;
            obj->GetUVCoords(uv_coords);
            scene.SetTexture(obj->GetTexturePath(), uv_coords);
        }
		if (_draw_bounding_box && obj->HasOuterFrame()) {
			MatrixXd bounding_box_vertices;
			MatrixXi bounding_box_topo;
			obj->GetFrameTopo(bounding_box_topo);
			obj->GetFrameVertices(bounding_box_vertices);
		}
    }
}

#include "Timer.h"
#include <chrono>

void Simulator::Processing(Scene &scene) {
    static int step_number = 0;
    static std::chrono::time_point<std::chrono::steady_clock> start_time;
    if (step_number == 0) {
        start_time = std::chrono::steady_clock::now();
    }
    step_number++;
    _time_stepper->Step(_time_step);
    
    if (step_number == 200) {
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
        spdlog::info("fps = {}", 200.0 / time_span.count());
        step_number = 0;
    }

    int id = 0;
    for (const auto& obj : _system->_objs) {
        scene.SelectData(_obj_id2scene_id[id++]);

		if (obj->IsRenderTopoUpdated()) {
			spdlog::info("render topo updated");
			MatrixXi topo;
			obj->GetRenderTopos(topo);
			scene.SetTopo(topo);
		}
        MatrixXd vertices;
		obj->GetRenderVertices(vertices);
        scene.SetMesh(vertices, obj->GetFrameRotation(), obj->GetFrameX());

		if (_draw_bounding_box && obj->HasOuterFrame()) {
			if (obj->IsFrameTopoUpdated()) {
				spdlog::info("bb topo updated");
				MatrixXi topo;
				obj->GetFrameTopo(topo);
				scene.SetBoundingBoxTopo(topo);
				// std::cerr << "BB topo" << std::endl << topo << std::endl;
			}
			MatrixXd bb_vertices;
			obj->GetFrameVertices(bb_vertices);
			scene.SetBoundingBoxMesh(bb_vertices, obj->GetFrameRotation(), obj->GetFrameX());
			// std::cerr << "BB vertices" << std::endl << bb_vertices << std::endl;
		}
    }
}