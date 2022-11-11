#include "Scene.hpp"
#include "RenderObject/RendererObject.hpp"
#include <iostream>
#include "spdlog/spdlog.h"

int Scene::AddMesh() {
    _meshes.push_back(RendererObject());
    _selected_data = _meshes.size() - 1;
    return _meshes.size() - 1;
}

void Scene::SelectData(int idx) {
    _selected_data = idx;
}

void Scene::SetMesh(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &topo, const Eigen::Matrix3d &R,
                    const Eigen::Vector3d &b) {
    if (_selected_data == -1) {
        spdlog::error("Try to access an invalid id");
        return;
    }
    _meshes[_selected_data].SetMesh(vertices, topo, R, b);
}

void Scene::Draw(Shader &shader) {
    for (auto& mesh : _meshes) {
        shader.SetFloat("rotation", mesh._rotation);
        shader.SetFloat("shift", mesh._shift);
        mesh.Bind();
        mesh.Draw();
    }
}

int Scene::AddMesh(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &topo, const Eigen::Matrix3d &R,
                   const Eigen::Vector3d &b) {
    int id = AddMesh();
    SetMesh(vertices, topo, R, b);
    return id;
}