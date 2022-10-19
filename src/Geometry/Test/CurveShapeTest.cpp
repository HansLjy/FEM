//
// Created by hansljy on 10/14/22.
//

#include "gtest/gtest.h"
#include "Curve/Curve.h"
#include "Shape.h"

TEST(ShapeTest, CurveShapeTest) {
    Vector3d start, end;
    start << 0, 0, -1;
    end << 0, 0, 1;
    const int num_segments = 1;
    const int num_vertices = 8 * num_segments;
    const int num_faces = 12 * num_segments;
    Curve curve(1, 1, start, end, num_segments);
    MatrixXd vertices;
    MatrixXi topo;
    curve.GetShape(vertices, topo);

    // std::cout << "Vertices: " << std::endl << vertices << std::endl;

    // Check whether the normal points "outwards"
    for (int i = 0; i < num_faces; i++) {
        RowVector3i vertex_id = topo.row(i);
        Eigen::RowVector3d vertex[3];
        for (int j = 0; j < 3; j++) {
            vertex[j] = vertices.row(vertex_id(j));
        }
        Vector3d normal = (vertex[1] - vertex[0]).cross(vertex[2] - vertex[0]);
        Vector3d center = (vertex[0] + vertex[1] + vertex[2]) / 3;
//        std::cout << "Normal = " << normal.transpose() << "\nCenter = " << center.transpose() << std::endl;
//        std::cout << "Normal * Center = " << normal.dot(center) << std::endl;
        EXPECT_GT(normal.dot(center), 0);
    }
}