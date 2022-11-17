//
// Created by hansljy on 11/17/22.
//

#ifndef FEM_PLAYER_H
#define FEM_PLAYER_H

#include "GUI/GUI.hpp"
#include "nlohmann/json.hpp"
using nlohmann::json;

class Player : public GUI {
public:
    void LoadAnimation(const std::string& config_path);
    void InitializeScene(Scene &scene) override;
    void Processing(Scene &scene) override;

protected:
    std::string _input_file_dir;
    int _start_itr_id, _cur_itr_id, _end_itr_id;    // -1 for default
    bool _is_loop;

    int _num_objects;
    int _num_iterations;

    std::vector<MatrixXi> _topos;
    std::vector<int> _obj_id2scene_id;

    glm::vec3 _camera_position, _camera_look, _camera_up;
    glm::vec3 _light_position, _light_ambient, _light_diffuse, _light_specular;
    float _light_Kc, _light_Kl, _light_Kq;
};

#endif //FEM_PLAYER_H
