#include "ExampleApp.hpp"
#include "EigenAll.h"

#include <iostream>

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
    MatrixXd vertices(8, 3);
    vertices << -0.5, -0.5, -0.5,
                +0.5, -0.5, -0.5,
                -0.5, +0.5, -0.5,
                +0.5, +0.5, -0.5, 
                -0.5, -0.5, +0.5,
                +0.5, -0.5, +0.5,
                -0.5, +0.5, +0.5,
                +0.5, +0.5, +0.5;
    
    Eigen::MatrixXi topo(12, 3);
    topo << 0,  4,  6,
            0,  6,  2,
            1,  3,  7,
            1,  7,  5,
            0,  1,  5,
            0,  5,  4,
            2,  6,  7,
            2,  7,  3,
            0,  2,  3,
            0,  3,  1,
            4,  5,  7,
            4,  7,  6;
    
    MatrixXd bb_vertices(12, 3);
    bb_vertices.block<8, 3>(0, 0) = vertices;
    bb_vertices.block<4, 3>(8, 0) = vertices.block<4, 3>(4, 0);
    bb_vertices.block<4, 1>(8, 2).array() += 1;

    Eigen::MatrixXi bb_topo(20, 3);
    bb_topo <<  0,  4,  6,
                0,  6,  2,
                1,  3,  7,
                1,  7,  5,
                0,  1,  5,
                0,  5,  4,
                2,  6,  7,
                2,  7,  3,
                0,  2,  3,
                0,  3,  1,
                4,  8, 10,
                4, 10,  6,
                5,  7, 11,
                5, 11,  9,
                4,  5,  9,
                4,  9,  8,
                6, 10, 11,
                6, 11,  7,
                8,  9, 11,
                8, 11, 10;

    MatrixXf uv_coords(8, 2);
    uv_coords <<
        0, 0,
        1, 0,
        2, 0,
        3, 0,
        0, 1,
        1, 1,
        2, 1,
        3, 1;

    auto id1 = scene.AddMesh();
    scene.SelectData(id1);
    scene.SetTexture(RENDERER_TEXTURE_PATH "/treetrunk.jpeg", uv_coords);
    // scene.SetTopo(topo);
    // scene.SetMesh(vertices, Matrix3d::Identity(), Vector3d::Zero());
	scene.SetBoundingBoxTopo(bb_topo);
	scene.SetBoundingBoxMesh(bb_vertices.array() * 2, Matrix3d::Identity(), Vector3d::Zero());

    // Matrix3d R = Matrix3d(AngleAxisd(EIGEN_PI / 12, Vector3d::UnitX()));
    // Vector3d b = (Vector3d() << 1, 1, 1).finished();
    // scene.AddMesh();
	// scene.SetTopo(topo);
	// scene.SetMesh(vertices, R, b);
}

void ExampleApp::Processing(Scene &scene) {
    
}