//
// Created by hansljy on 10/19/22.
//

#include "Curve.h"
#include "CurveShape/CurveShape.h"

VectorXd Curve::GetX(const Vector3d &start, const Vector3d &end, int num_segments){
    VectorXd x(3 * (num_segments + 1));
    Vector3d delta = (end - start) / num_segments;
    Vector3d current = start;
    for (int i = 0, j = 0; i <= num_segments; i++, j += 3, current += delta) {
        x.segment<3>(j) = current;
    }
    return x;
}

Curve::Curve(double total_mass, double alpha, const Eigen::Vector3d &start, const Eigen::Vector3d &end,
             int num_segments) : Curve(total_mass, alpha, GetX(start, end, num_segments)) {}

Curve::Curve(double total_mass, double alpha, const Eigen::VectorXd &x) : ShapedObject(x, CurveShape()), _alpha(alpha), _num_points(x.size() / 3) {
     _x_rest = x;
     _mass.resize(_num_points);
     _mass.setConstant(total_mass / _num_points);
     _mass_sparse.resize(_num_points * 3);
     _mass_sparse.setConstant(total_mass / _num_points);
     _rest_length.resize(_num_points - 1);
     for (int i = 0, j = 0; i < _num_points - 1; i++, j += 3) {
         _rest_length(i) = (_x_rest.block<3, 1>(j + 3, 0) - _x_rest.block<3, 1>(j, 0)).norm();
     }
     _voronoi_length.resize(_num_points);
     for (int i = 1; i < _num_points - 1; i++) {
         _voronoi_length(i) = (_rest_length(i - 1) + _rest_length(i)) / 2;
     }
     _voronoi_length(0) = _rest_length(0) / 2;
     _voronoi_length(_num_points - 1) = _rest_length(_num_points - 2) / 2;
}

#include "JsonUtil.h"

Curve::Curve(const nlohmann::json &config)
    : Curve(config["mass"], config["alpha"], Json2Vec(config["start"]),
            Json2Vec(config["end"]), config["segments"]) {}

void Curve::GetMass(COO &coo, int x_offset, int y_offset) const {
    for (int i = 0, j = 0; i < _num_points; i++, j += 3) {
        coo.push_back(Tripletd(x_offset + j, y_offset + j, _mass(i)));
        coo.push_back(Tripletd(x_offset + j + 1, y_offset + j + 1, _mass(i)));
        coo.push_back(Tripletd(x_offset + j + 2, y_offset + j + 2, _mass(i)));
    }
}

int Curve::GetConstraintSize() const {
    return Object::GetConstraintSize();
}

VectorXd Curve::GetInnerConstraint(const VectorXd &x) const {
    return Object::GetInnerConstraint(x);
}

void Curve::GetInnerConstraintGradient(const VectorXd &x, COO &coo, int x_offset, int y_offset) const {
    Object::GetInnerConstraintGradient(x, coo, x_offset, y_offset);
}
