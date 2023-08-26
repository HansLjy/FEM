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
	void SetBoundingBoxTopo(const MatrixXi& topo);

    void SetMesh(const MatrixXd& vertices, const Matrix3d& R, const Vector3d& b);
    void SetBoundingBoxMesh(const MatrixXd& vertices, const Matrix3d& R, const Vector3d& b);

    void SetTexture(const std::string& texture_path, const MatrixXf& uv_coords);

    void Draw(Shader& shader) const;
	void DrawBoundingBox(Shader& shader) const;

    glm::mat3 _rotation;
    glm::vec3 _shift;

private:
	using RowMajorVertexMatrix = Eigen::Matrix<float, Eigen::Dynamic, 8, Eigen::RowMajor>;
	using RowMajorTopoMatrix = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
    bool _use_texture = false;
    unsigned int _VAO = 0;
    unsigned int _bb_VAO = 0;
    unsigned int _texture_id = 0;
    RowMajorVertexMatrix _vertex_array_data;
    RowMajorVertexMatrix _bb_vertex_array_data;
    RowMajorTopoMatrix _topo;
    RowMajorTopoMatrix _bb_topo;

	template<unsigned int RendererObject::*VAO, RowMajorVertexMatrix RendererObject::*vertex_data>
    void BindVertexArrayTemplate();
	template<RowMajorTopoMatrix RendererObject::*topo_data, RowMajorVertexMatrix RendererObject::*vertex_data>
    void SetTopoTemplate(const MatrixXi& topo);
	template<unsigned int RendererObject::*VAO, RowMajorTopoMatrix RendererObject::*topo_data, RowMajorVertexMatrix RendererObject::*vertex_data>
    void SetMeshTemplate(const MatrixXd& vertices, const Matrix3d& R, const Vector3d& b);
};

template<
	RendererObject::RowMajorTopoMatrix RendererObject::*topo_data,
	RendererObject::RowMajorVertexMatrix RendererObject::*vertex_data
>
void RendererObject::SetTopoTemplate(const MatrixXi& topo) {
	this->*topo_data = topo;
	(this->*vertex_data).resize(topo.size(), 8);
}

template<
	unsigned int RendererObject::*VAO,
	RendererObject::RowMajorTopoMatrix RendererObject::*topo_data,
	RendererObject::RowMajorVertexMatrix RendererObject::*vertex_data
>
void RendererObject::SetMeshTemplate(const MatrixXd& vertices, const Matrix3d& R, const Vector3d& b) {
	// TODO: make this more efficient
	int num_triangles = (this->*topo_data).rows();
    for (int i = 0, cnt_rows = 0; i < num_triangles; i++, cnt_rows += 3) {
        Eigen::RowVector3i indices = (this->*topo_data).row(i);
        Eigen::RowVector3f vertex[3];
        for (int j = 0; j < 3; j++) {
            vertex[j] = vertices.row(indices[j]).cast<float>();
        }
        Eigen::RowVector3f normal = ((vertex[1] - vertex[0]).cross(vertex[2] - vertex[0])).cast<float>();

        for (int j = 0; j < 3; j++) {
            (this->*vertex_data).block<1, 3>(cnt_rows + j, 0) = vertex[j];
            (this->*vertex_data).block<1, 3>(cnt_rows + j, 3) = normal;
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            _rotation[i][j] = R(j, i);  // This is because _rotation[i] means ith column,
                                        // not row. GLM is fucking shit
        }
        _shift[i] = b(i);
    }

    BindVertexArrayTemplate<VAO, vertex_data>();
}

template<
	unsigned int RendererObject::*VAO, 
	RendererObject::RowMajorVertexMatrix RendererObject::*vertex_data
>
void RendererObject::BindVertexArrayTemplate() {
	unsigned int VBO;
    glGenBuffers(1, &VBO);

    glBindVertexArray(this->*VAO);
    
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, (this->*vertex_data).size() * sizeof(float), (this->*vertex_data).data(), GL_DYNAMIC_DRAW);

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

#endif

