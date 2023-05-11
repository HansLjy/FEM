#include "FixedShape.hpp"
#include "JsonUtil.h"

MatrixXd GetRectangleVertices(const json &config);
MatrixXi GetRectangleFaceTopos(const json& config);
MatrixXi GetRectangleEdgeTopos(const json& config);

namespace {
    const bool rectangle_registered =
           FixedShapeFactory::Instance()->RegisterVerticesGenerator("rectangle", GetRectangleVertices)
        && FixedShapeFactory::Instance()->RegisterEdgeTopoGenerator("rectangle", GetRectangleEdgeTopos)
        && FixedShapeFactory::Instance()->RegisterFaceTopoGenerator("rectangle", GetRectangleFaceTopos);
}

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
MatrixXd GetRectangleVertices(const json &config) {
    const Vector3d max_point = Json2Vec(config["max-point"]);
    const Vector3d min_point = Json2Vec(config["min-point"]);
    MatrixXd vertices(8, 3);
    for (int i = 0; i < 8; i++) {
        vertices.row(i) <<
            ((i & 1) ? max_point.x() : min_point.x()),
            ((i & 2) ? max_point.y() : min_point.y()),
            ((i & 4) ? max_point.z() : min_point.z());
    }
    return vertices;
}

MatrixXi GetRectangleEdgeTopos(const json& config) {
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

MatrixXi GetRectangleFaceTopos(const json& config) {
    Matrix<int, Dynamic, Dynamic, Eigen::RowMajor> topo(12, 3);
    topo <<
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
    return topo;
}
