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
    _topo = topo;
    _vertex_array_data.resize(_topo.size(), 8);
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

void RendererObject::SetMesh(const Eigen::MatrixXd &vertices, const Eigen::Matrix3d &R, const Eigen::Vector3d &b) {
    // TODO: make this more efficient
    int num_triangles = _topo.rows();
    for (int i = 0, cnt_rows = 0; i < num_triangles; i++, cnt_rows += 3) {
        Eigen::RowVector3i indices = _topo.row(i);
        Eigen::RowVector3f vertex[3];
        for (int j = 0; j < 3; j++) {
            vertex[j] = vertices.row(indices[j]).cast<float>();
        }
        Eigen::RowVector3f normal = ((vertex[1] - vertex[0]).cross(vertex[2] - vertex[0])).cast<float>();

        for (int j = 0; j < 3; j++) {
            _vertex_array_data.block<1, 3>(cnt_rows + j, 0) = vertex[j];
            _vertex_array_data.block<1, 3>(cnt_rows + j, 3) = normal;
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            _rotation[i][j] = R(j, i);  // This is because _rotation[i] means ith column,
                                        // not row. GLM is fucking shit
        }
        _shift[i] = b(i);
    }

    BindVertexArray();
}

void RendererObject::BindVertexArray() {
    unsigned int VBO;
    glGenBuffers(1, &VBO);

    glBindVertexArray(_VAO);
    
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, _vertex_array_data.size() * sizeof(float), _vertex_array_data.data(), GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glDeleteBuffers(1, &VBO);
}

void RendererObject::Draw(Shader& shader) const {
    shader.SetFloat("rotation", _rotation);
    shader.SetFloat("shift", _shift);
    shader.SetInt("useTexture", _use_texture);
    if (_use_texture) {
        glBindTexture(GL_TEXTURE_2D, _texture_id);
    }
    glBindVertexArray(_VAO);
    glDrawArrays(GL_TRIANGLES, 0, _vertex_array_data.rows());
}

RendererObject::~RendererObject() {
    if (_VAO != 0) {
        glDeleteVertexArrays(1, &_VAO);
    }
}

RendererObject::RendererObject(const RendererObject& mesh)
    : _rotation(mesh._rotation),
      _shift(mesh._shift),
      _use_texture(mesh._use_texture), 
      _texture_id(mesh._texture_id), 
      _topo(mesh._topo) {
    _vertex_array_data = mesh._vertex_array_data;
    glGenVertexArrays(1, &_VAO);
    BindVertexArray();
}