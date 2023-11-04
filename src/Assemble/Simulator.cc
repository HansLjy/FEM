//
// Created by hansljy on 10/11/22.
//

#include "Simulator.h"
#include "JsonUtil.h"
#include "Pattern.h"
#include "FileIO.hpp"
#include <string>

template<>
Caster<Renderable>* Caster<Renderable>::_the_factory = nullptr;

using nlohmann::json;

void Simulator::LoadScene(const std::string &filename) {
    json config;
    std::ifstream config_file(filename);
    config_file >> config;
	config_file.close();
    auto simulation_config = config["simulation-config"];
    _duration = simulation_config["duration"];
    _time_step = simulation_config["time-step"];
	_dump_mesh = simulation_config["dump-mesh"];
	if (_dump_mesh) {
		_mesh_path = std::string(simulation_config["mesh-path"]);
	}

	_dump_coord = simulation_config["dump-coord"];
	_pickup_from_dump = simulation_config["pickup-from-dump"];
	if (_pickup_from_dump) {
		_pickup_frame_number = simulation_config["pickup-frame-number"];
	}


	const std::string time_stepper_config_filepath = config["time-stepper-config"];
	std::ifstream time_stepper_config_file(std::string(CONFIG_PATH) + "/time-stepper" + time_stepper_config_filepath);
    const json time_stepper_config = json::parse(time_stepper_config_file);
	time_stepper_config_file.close();
    _time_stepper = Factory<TimeStepper>::GetInstance()->GetProduct(
        time_stepper_config["type"],
        time_stepper_config
    );

    const std::string scene_config_filepath = std::string(CONFIG_PATH) + "/scene" + std::string(config["scene-config"]);
    std::ifstream system_config_file(scene_config_filepath);
    const json scene_config = json::parse(system_config_file);
    system_config_file.close();

	_time_stepper->BindSystem(scene_config);

	const std::string renderer_config_filepath = std::string(CONFIG_PATH) + "/renderer" + std::string(config["renderer-config"]);
	std::ifstream renderer_config_file(renderer_config_filepath);
	const json renderer_config = json::parse(renderer_config_file);
    const auto& camera_config_json = renderer_config["camera"];
    _camera_position = Json2GlmVec3(camera_config_json["position"]);
    _camera_look = Json2GlmVec3(camera_config_json["look"]);
    _camera_up = Json2GlmVec3(camera_config_json["up"]);

    const auto& light_config_json = renderer_config["light"];
    _light_position = Json2GlmVec3(light_config_json["position"]);
    _light_ambient = Json2GlmVec3(light_config_json["ambient"]);
    _light_diffuse = Json2GlmVec3(light_config_json["diffuse"]);
    _light_specular = Json2GlmVec3(light_config_json["specular"]);
    _light_Kc = light_config_json["Kc"];
    _light_Kl = light_config_json["Kl"];
    _light_Kq = light_config_json["Kq"];

	if (_pickup_from_dump) {
		_time_stepper->PickCoord(_pickup_frame_number);
	}

    spdlog::info("Scene loaded");
}

void Simulator::Simulate(const std::string& output_dir) {
    double current_time = 0;

	std::ofstream topo_file(output_dir + "/topo");
	const auto& renderable_object = _time_stepper->GetRenderObjects();
	for (const auto& obj : renderable_object) {
        MatrixXi topo(obj.GetRenderFaceNum(), 3);
		obj.GetRenderTopos(topo);
        write_binary(topo_file, topo);
	}
	topo_file.close();

    int itr_id = 0;
    while(current_time < _duration) {
        int obj_id = 0;
		std::ofstream mesh_file(output_dir + "/itr" + std::to_string(itr_id));
        for (const auto& obj : renderable_object) {
        	MatrixXd vertices(obj.GetRenderVertexNum(), 3);
			obj.GetRenderVertices(vertices);
            write_binary(mesh_file, vertices);
        }
		mesh_file.close();
        itr_id++;

		if (_dump_mesh) {
			int obj_id = 0;
			for (const auto& obj : renderable_object) {
				MatrixXd vertices(obj.GetRenderVertexNum(), 3);
				MatrixXi topo(obj.GetRenderFaceNum(), 3);

				obj.GetRenderTopos(topo);
				obj.GetRenderVertices(vertices);

				FileIOUtils::WriteMesh(_mesh_path + "/" + std::to_string(obj_id) + "_itr" + std::to_string(itr_id) + ".obj", vertices, topo);
				obj_id++;
			}
		}

		if (_dump_coord) {
			_time_stepper->DumpCoord(itr_id);
		}

		_time_stepper->Step(_time_step);
        current_time += _time_step;
        spdlog::info("Current time: {}", current_time);
    }
    json config;
    config["number-of-objects"] = renderable_object.size();
    config["number-of-iterations"] = itr_id;
    std::ofstream config_file(output_dir + "/" + "config.json");
    config_file << config.dump();
    config_file.close();
}

Simulator::~Simulator() {
    delete _time_stepper;
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

	const auto& renderable_object = _time_stepper->GetRenderObjects();
    for (const auto& obj : renderable_object) {
        MatrixXd vertices(obj.GetRenderVertexNum(), 3);
        MatrixXi topo(obj.GetRenderFaceNum(), 3);

		obj.GetRenderTopos(topo);
		obj.GetRenderVertices(vertices);

        _obj_id2scene_id.push_back(scene.AddMesh(vertices, topo, Matrix3d::Identity(), Vector3d::Zero()));
        if (obj.IsUsingTexture()) {
            MatrixXf uv_coords;
            obj.GetUVCoords(uv_coords);
            scene.SetTexture(obj.GetTexturePath(), uv_coords);
        }
    }
	if (_dump_mesh) {
		int obj_id = 0;
		for (const auto& obj : renderable_object) {
			MatrixXd vertices(obj.GetRenderVertexNum(), 3);
			MatrixXi topo(obj.GetRenderFaceNum(), 3);

			obj.GetRenderTopos(topo);
			obj.GetRenderVertices(vertices);

			FileIOUtils::WriteMesh(_mesh_path + "/" + std::to_string(obj_id) + "_itr0.obj", vertices, topo);
			obj_id++;
		}
	}
}

#include "Timer.h"
#include <chrono>

void Simulator::Processing(Scene &scene) {
	static int itr = 0;
    static int step_number = 0;
    static std::chrono::time_point<std::chrono::steady_clock> start_time;
    if (step_number == 0) {
        start_time = std::chrono::steady_clock::now();
    }
    step_number++;
    
	_time_stepper->Step(_time_step);
	itr++;
    
    if (step_number == 200) {
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
        spdlog::info("fps = {}", 200.0 / time_span.count());
        step_number = 0;
    }

    int id = 0;
	const auto& renderable_object = _time_stepper->GetRenderObjects();
    for (const auto& obj : renderable_object) {
        scene.SelectData(_obj_id2scene_id[id++]);

		if (obj.IsRenderTopoUpdated()) {
			spdlog::info("render topo updated");
			MatrixXi topo(obj.GetRenderFaceNum(), 3);
			obj.GetRenderTopos(topo);
			scene.SetTopo(topo);
		}
        MatrixXd vertices(obj.GetRenderVertexNum(), 3);
		obj.GetRenderVertices(vertices);
        scene.SetMesh(vertices, Matrix3d::Identity(), Vector3d::Zero());
    }

	if (_dump_mesh) {
		int obj_id = 0;
		for (const auto& obj : renderable_object) {
			MatrixXd vertices(obj.GetRenderVertexNum(), 3);
			MatrixXi topo(obj.GetRenderFaceNum(), 3);

			obj.GetRenderTopos(topo);
			obj.GetRenderVertices(vertices);

			FileIOUtils::WriteMesh(_mesh_path + "/" + std::to_string(obj_id) + "_itr" + std::to_string(step_number) + ".obj", vertices, topo);
			obj_id++;
		}
	}
	if (_dump_coord) {
		_time_stepper->DumpCoord(itr);
	}

	spdlog::info("#iterations = {}", itr);
}