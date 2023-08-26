#include "RendererObject.hpp"
#include <cstdlib>
#include "spdlog/spdlog.h"
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// TODO: make this more elegant
std::map<std::string, unsigned int> texture_pool;

RendererObject::RendererObject() {
    glGenVertexArrays(1, &_VAO);
}

void RendererObject::SetTopo(const MatrixXi &topo) {
	SetTopoTemplate<&RendererObject::_topo, &RendererObject::_vertex_array_data>(topo);
}

void RendererObject::SetBoundingBoxTopo(const MatrixXi &topo) {
	SetTopoTemplate<&RendererObject::_bb_topo, &RendererObject::_bb_vertex_array_data>(topo);
}

void RendererObject::SetMesh(const Eigen::MatrixXd &vertices, const Eigen::Matrix3d &R, const Eigen::Vector3d &b) {
	SetMeshTemplate<&RendererObject::_VAO, &RendererObject::_topo, &RendererObject::_vertex_array_data>(vertices, R, b);
}

void RendererObject::SetBoundingBoxMesh(const Eigen::MatrixXd &vertices, const Eigen::Matrix3d &R, const Eigen::Vector3d &b) {
	SetMeshTemplate<&RendererObject::_bb_VAO, &RendererObject::_bb_topo, &RendererObject::_bb_vertex_array_data>(vertices, R, b);
}

void RendererObject::SetTexture(const std::string &texture_path, const MatrixXf& uv_coords) {
    _use_texture = true;

    if (texture_pool.find(texture_path) == texture_pool.end()) {
        // new texture
        glGenTextures(1, &_texture_id);
        glBindTexture(GL_TEXTURE_2D, _texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);	
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
        int width, height, nrChannels;
        unsigned char *data = stbi_load(texture_path.c_str(), &width, &height, &nrChannels, 0); 

        if (data == nullptr) {
            throw std::logic_error("texture not found");
        }
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        stbi_image_free(data);

        texture_pool.insert(std::make_pair(texture_path, _texture_id));
    } else {
        _texture_id = texture_pool.find(texture_path)->second;
    }

    for (int i = 0, cnt_rows = 0; i < _topo.rows(); i++, cnt_rows += 3) {
        Eigen::RowVector3i indices = _topo.row(i);
        for (int j = 0; j < 3; j++) {
            _vertex_array_data.block<1, 2>(cnt_rows + j, 6) = uv_coords.row(indices[j]);
        }
    }
}

void RendererObject::Draw(Shader& shader) const {
    shader.SetFloat("rotation", _rotation);
    shader.SetFloat("shift", _shift);
    shader.SetInt("useTexture", _use_texture);
	shader.SetFloat("alpha", 1);
    if (_use_texture) {
        glBindTexture(GL_TEXTURE_2D, _texture_id);
    }
    glBindVertexArray(_VAO);
    glDrawArrays(GL_TRIANGLES, 0, _vertex_array_data.rows());
}

void RendererObject::DrawBoundingBox(Shader& shader) const {
    shader.SetFloat("rotation", _rotation);
    shader.SetFloat("shift", _shift);
	shader.SetFloat("alpha", 0.1);
    glBindVertexArray(_bb_VAO);
    glDrawArrays(GL_TRIANGLES, 0, _bb_vertex_array_data.rows());
}

RendererObject::~RendererObject() {
    if (_VAO != 0) {
        glDeleteVertexArrays(1, &_VAO);
    }
    if (_bb_VAO != 0) {
        glDeleteVertexArrays(1, &_bb_VAO);
    }
}

RendererObject::RendererObject(const RendererObject& rhs)
    : _rotation(rhs._rotation),
      _shift(rhs._shift),
      _use_texture(rhs._use_texture), 
      _texture_id(rhs._texture_id),
	  _vertex_array_data(rhs._vertex_array_data),
	  _bb_vertex_array_data(rhs._bb_vertex_array_data),
      _topo(rhs._topo),
	  _bb_topo(rhs._bb_topo)
	  {
    glGenVertexArrays(1, &_VAO);
    BindVertexArrayTemplate<&RendererObject::_VAO, &RendererObject::_vertex_array_data>();
	glGenVertexArrays(1, &_bb_VAO);
    BindVertexArrayTemplate<&RendererObject::_bb_VAO, &RendererObject::_bb_vertex_array_data>();
}