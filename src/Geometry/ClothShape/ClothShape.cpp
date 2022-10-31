//
// Created by hansljy on 10/27/22.
//

#include "ClothShape.h"
#include "Cloth/Cloth.h"

DEFINE_CLONE(Shape, ClothShape)

void ClothShape::GetSurface(const Object &object, Eigen::MatrixXd &vertices, Eigen::MatrixXi &topos) const {
    auto& cloth = dynamic_cast<const Cloth&>(object);
    const int num_points = cloth._num_points;
    vertices.resize(num_points, 3);
    for (int i = 0, j = 0; i < num_points; i++, j += 3) {
        vertices.row(i) = cloth._x.segment<3>(j);
    }
    topos = cloth._topo;
}