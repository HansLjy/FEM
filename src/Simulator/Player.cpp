//
// Created by hansljy on 11/17/22.
//

#include "Player.h"
#include "fstream"
#include "JsonUtil.h"
#include <fstream>
#include <string>

void Player::LoadAnimation(const std::string &config_path) {
    std::ifstream config_file(config_path);
    const json config_json = json::parse(config_file);
    _input_file_dir = config_json["input-file-directory"];
    _input_file_dir = std::string(OUTPUT_PATH) + "/" + _input_file_dir;
    _start_itr_id = config_json["start-iteration-number"];
    _cur_itr_id = _start_itr_id;
    _end_itr_id = config_json["end-iteration-number"];
    _is_loop = config_json["is-loop"];

    const auto& camera_config_json = config_json["camera"];
    _camera_position = Json2GlmVec3(camera_config_json["position"]);
    _camera_look = Json2GlmVec3(camera_config_json["look"]);
    _camera_up = Json2GlmVec3(camera_config_json["up"]);

    const auto& light_config_json = config_json["light"];
    _light_position = Json2GlmVec3(light_config_json["position"]);
    _light_ambient = Json2GlmVec3(light_config_json["ambient"]);
    _light_diffuse = Json2GlmVec3(light_config_json["diffuse"]);
    _light_specular = Json2GlmVec3(light_config_json["specular"]);
    _light_Kc = light_config_json["Kc"];
    _light_Kl = light_config_json["Kl"];
    _light_Kq = light_config_json["Kq"];
}

void Player::InitializeScene(Scene &scene) {
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

    std::ifstream config_file(_input_file_dir + "/" + "config.json");
    const json config_json = json::parse(config_file);
    config_file.close();

    _num_objects = config_json["number-of-objects"];
    _num_iterations = config_json["number-of-iterations"];

    _end_itr_id = _end_itr_id < 0 ? _num_iterations + _end_itr_id + 1 : _end_itr_id;

    MatrixXi topo;
	std::ifstream topo_file(_input_file_dir + "/topo");
    for (int i = 0; i < _num_objects; i++) {
        read_binary(topo_file, topo);
        _topos.push_back(topo);
        _obj_id2scene_id.push_back(scene.AddMesh());
    }
}

#include "spdlog/spdlog.h"
#include "iostream"

void Player::Processing(Scene &scene) {
	std::ifstream itr_file(_input_file_dir + "/itr" + std::to_string(_cur_itr_id));
    for (int obj_id = 0; obj_id < _num_objects; obj_id++) {
        MatrixXd vertices;
        MatrixXd rotation, translation;
        read_binary(itr_file, vertices);
        read_binary(itr_file, rotation);
        read_binary(itr_file, translation);
        scene.SelectData(_obj_id2scene_id[obj_id]);
        scene.SetMesh(vertices, _topos[obj_id], rotation, translation);
    }
	itr_file.close();

    spdlog::info("Current iteration: {}", _cur_itr_id);

    if (++_cur_itr_id >= _end_itr_id) {
        _cur_itr_id = _is_loop ? _start_itr_id : _end_itr_id;
    }
}