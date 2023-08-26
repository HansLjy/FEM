#ifndef SCENE_HPP
#define SCENE_HPP

#include "EigenAll.h"
#include <vector>
#include "RenderObject/RendererObject.hpp"
#include "Shader/Shader.hpp"

class Scene {
public:
    /**
     * @brief Draw the whole scene
     */
    void Draw(Shader& shader);
    void DrawBoundingBox(Shader& shader);

    /**
     * @brief Add a new slot for meshes into the scene
     * @return int The id for the new slot
     * @note This method will automatically select the
     *       newest mesh afterwards
     */
    int AddMesh();

    /**
     * @brief Select a specific mesh
     * @param idx The id for the meshes
     */
    void SelectData(int idx);

    void SetTexture(const std::string& texture_path, const MatrixXf& uv_coords);

    void SetMesh(const MatrixXd& vertices, const Matrix3d& R, const Vector3d& b);
    void SetBoundingBoxMesh(const MatrixXd& vertices, const Matrix3d& R, const Vector3d& b);
    void SetTopo(const MatrixXi& topo);
    void SetBoundingBoxTopo(const MatrixXi& topo);

    int AddMesh(const MatrixXd& vertices, const MatrixXi& topo, const Matrix3d& R, const Vector3d& b);

private:
    std::vector<RendererObject> _meshes;
    int _selected_data = -1;
};

#endif