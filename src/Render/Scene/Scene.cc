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

void Scene::SetMesh(const Eigen::MatrixXd &vertices, const Eigen::Matrix3d &R, const Eigen::Vector3d &b) {
    if (_selected_data == -1) {
        spdlog::error("Try to access an invalid id");
        return;
    }
    _meshes[_selected_data].SetMesh(vertices, R, b);
}

void Scene::SetBoundingBoxMesh(const MatrixXd &vertices, const Matrix3d &R, const Vector3d &b) {
    if (_selected_data == -1) {
        spdlog::error("Try to access an invalid id");
        return;
    }
    _meshes[_selected_data].SetBoundingBoxMesh(vertices, R, b);
}

void Scene::SetTexture(const std::string &texture_path, const MatrixXf &uv_coords) {
    if (_selected_data == -1) {
        throw std::logic_error("Try to access an invalid id");
    }
    _meshes[_selected_data].SetTexture(texture_path, uv_coords);
}

void Scene::SetTopo(const MatrixXi &topo) {
    if (_selected_data == -1) {
        throw std::logic_error("Try to access an invalid id");
    }
    _meshes[_selected_data].SetTopo(topo);
}

void Scene::SetBoundingBoxTopo(const MatrixXi &topo) {
    if (_selected_data == -1) {
        throw std::logic_error("Try to access an invalid id");
    }
    _meshes[_selected_data].SetBoundingBoxTopo(topo);
}

void Scene::Draw(Shader &shader) {
    for (const auto& mesh : _meshes) {
        mesh.Draw(shader);
    }
}

void Scene::DrawBoundingBox(Shader &shader) {
    for (const auto& mesh : _meshes) {
        mesh.DrawBoundingBox(shader);
    }
}

int Scene::AddMesh(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &topo, const Eigen::Matrix3d &R, const Eigen::Vector3d &b) {
    int id = AddMesh();
    SetTopo(topo);
    SetMesh(vertices, R, b);
    return id;
}