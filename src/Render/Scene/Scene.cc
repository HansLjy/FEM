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

void Scene::SetMesh(const MatrixXd &vertices, const MatrixXi &topo) {
    if (_selected_data == -1) {
        spdlog::error("Try to access an invalid id");
        return;
    }
    _meshes[_selected_data].SetMesh(vertices, topo);
}

void Scene::Draw() {
    for (auto& mesh : _meshes) {
        mesh.Bind();
        mesh.Draw();
    }
}

int Scene::AddMesh(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &topo) {
    int id = AddMesh();
    SetMesh(vertices, topo);
    return id;
}