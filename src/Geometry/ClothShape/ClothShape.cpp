//
// Created by hansljy on 10/27/22.
//

#include "ClothShape.h"
#include "Cloth/Cloth.h"

DEFINE_CLONE(Shape, ClothShape)

void ClothShape::GetSurface(const Object &object, Eigen::MatrixXd &vertices, Eigen::MatrixXi &topos) const {
    auto cloth = dynamic_cast<const Cloth&>(object);
    vertices = cloth._x;
    topos = cloth._topo;
}