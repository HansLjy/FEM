#ifndef VERTEX_ARRAY_HPP
#define VERTEX_ARRAY_HPP

#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "EigenAll.h"
#include "glm/glm.hpp"
#include "Shader/Shader.hpp"

class RendererObject {
public:
    RendererObject();
    ~RendererObject();

    RendererObject(const RendererObject& mesh);
    RendererObject& operator=(const RendererObject& mesh) = delete;

    void SetTopo(const MatrixXi& topo);

    void SetMesh(const MatrixXd& vertices, const Matrix3d& R, const Vector3d& b);

    void SetTexture(const std::string& texture_path, const MatrixXf& uv_coords);

    void Draw(Shader& shader) const;

    glm::mat3 _rotation;
    glm::vec3 _shift;

private:
    bool _use_texture = false;
    unsigned int _VAO = 0;
    unsigned int _texture_id = 0;
    Eigen::Matrix<float, Eigen::Dynamic, 8, Eigen::RowMajor> _vertex_array_data;
    Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> _topo;

    void BindVertexArray();
};

#endif

