#include "RectangleCollisionShape.h"
#include "JsonUtil.h"

RectangleCollisionShape::RectangleCollisionShape(const json& config)
    : RectangleCollisionShape(Json2Vec(config["min-point"]), Json2Vec(config["max-point"])) {}

RectangleCollisionShape::RectangleCollisionShape(const Vector3d& min_point, const Vector3d& max_point)
    : FixedCollisionShape(GenerateVertices(min_point, max_point), SparseMatrixXd(24, 0), GenerateEdgeTopo(), GenerateFaceTopo()) {}

/**
 * vertices indices
 *      4 --- 6
 *     /     / |
 *    5 --- 7  |
 *    |     |  2
 *    |     | /
 *    1 --- 3 
 * the unseen number is 0
 * min-point and max-point refer to 0 and 7
 */

MatrixXd RectangleCollisionShape::GenerateVertices(const Vector3d& min_point, const Vector3d& max_point) {
    MatrixXd vertices(8, 3);
    for (int i = 0; i < 8; i++) {
        vertices.row(i) <<
            ((i & 1) ? max_point.x() : min_point.x()),
            ((i & 2) ? max_point.y() : min_point.y()),
            ((i & 4) ? max_point.z() : min_point.z());
    }
    return vertices;
}

MatrixXi RectangleCollisionShape::GenerateEdgeTopo() {
    Matrix<int, Dynamic, Dynamic, Eigen::RowMajor> edge_topo(12, 2);
    edge_topo <<
        0, 4,
        0, 1,
        1, 5,
        5, 4,
        4, 6,
        5, 7,
        1, 3,
        0, 2,
        2, 3,
        3, 7,
        7, 6,
        6, 2;
    return edge_topo;
}

MatrixXi RectangleCollisionShape::GenerateFaceTopo() {
    Matrix<int, Dynamic, Dynamic, Eigen::RowMajor> face_topo(12, 3);
    face_topo <<
        0, 5, 4,
        0, 1, 5,
        2, 6, 7,
        2, 7, 3,
        7, 6, 4,
        7, 4, 5,
        3, 1, 0,
        3, 0, 2,
        7, 5, 1,
        7, 1, 3,
        6, 2, 0,
        6, 0, 4;
    return face_topo;
}

