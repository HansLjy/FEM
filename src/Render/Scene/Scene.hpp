#ifndef SCENE_HPP
#define SCENE_HPP

#include "EigenAll.h"
#include <vector>
#include "RenderObject/RendererObject.hpp"

class Scene {
public:
    /**
     * @brief Draw the whole scene
     */
    void Draw();

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

    /**
     * @brief Set the Mesh for the seleted mesh
     * 
     */
    void SetMesh(const MatrixXd& vertices, const MatrixXi& topo);

    int AddMesh(const MatrixXd& vertices, const MatrixXi& topo);

private:
    std::vector<RendererObject> _meshes;
    int _selected_data = -1;
};

#endif