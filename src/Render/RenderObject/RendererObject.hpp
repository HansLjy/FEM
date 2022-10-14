#ifndef VERTEX_ARRAY_HPP
#define VERTEX_ARRAY_HPP

#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "EigenAll.h"

class RendererObject {
public:
    RendererObject();
    ~RendererObject();

    RendererObject(const RendererObject& mesh);
    RendererObject& operator=(const RendererObject& mesh) = delete;

    void SetMesh(const MatrixXd& vertices, const MatrixXi& topos);

    void Bind();
    void Draw();

private:
    unsigned int _VAO = 0;
    Eigen::Matrix<float, Eigen::Dynamic, 6, Eigen::RowMajor> _vertex_array_data;

    void BindVertexArray();
};

#endif

