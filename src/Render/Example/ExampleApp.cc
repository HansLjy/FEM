#include "ExampleApp.hpp"
#include "EigenAll.h"

void ExampleApp::InitializeScene(Scene &scene) {
    SetCamera(
        glm::vec3(5.0f, 0.0f, 5.0f),
        glm::vec3(-1.0f, 0.0f, -1.0f),
        glm::vec3(0.0f, 0.0f, 1.0f)
    );
    SetLight(
        glm::vec3(0.0f, 10.0f, 0.0f),
        glm::vec3(0.2f, 0.2f, 0.2f),
        glm::vec3(0.5f, 0.5f, 0.5f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        1.0f, 0.35f, 0.44f
    );
    id1 = scene.AddMesh();
}

void ExampleApp::Processing(Scene &scene) {
    MatrixXd vertices(8, 3);
    vertices << +0.5, +0.5, +0.5, 
                -0.5, +0.5, +0.5,
                +0.5, -0.5, +0.5,
                -0.5, -0.5, +0.5,
                +0.5, +0.5, -0.5, 
                -0.5, +0.5, -0.5,
                +0.5, -0.5, -0.5,
                -0.5, -0.5, -0.5;

    vertices.array() += 1;

    Eigen::MatrixXi topo(12, 3);
    topo << 0, 1, 3,
            0, 3, 2,
            0, 5, 1,
            0, 4, 5,
            0, 2, 4,
            2, 6, 4,
            2, 3, 7,
            2, 7, 6,
            4, 6, 7,
            4, 7, 5,
            1, 5, 3,
            3, 5, 7;

    scene.SelectData(id1);
    scene.SetMesh(vertices, topo);


}