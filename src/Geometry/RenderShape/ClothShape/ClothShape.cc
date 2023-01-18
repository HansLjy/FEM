//
// Created by hansljy on 10/27/22.
//

#include "ClothShape.h"
#include "Object/Cloth.h"

void ClothShape::Bind(const Object &obj) {
	_cloth = dynamic_cast<const Cloth*>(&obj);
}

void ClothShape::GetSurface(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topos) const {
    const int num_points = _cloth->_num_points;
    vertices.resize(num_points, 3);
    for (int i = 0, j = 0; i < num_points; i++, j += 3) {
        vertices.row(i) = _cloth->_x.segment<3>(j);
    }
    topos = _cloth->_topo;
}