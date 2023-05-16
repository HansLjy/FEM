//
// Created by hansljy on 11/11/22.
//

#pragma once

#include "RenderShape.hpp"

class TreeTrunkShape : public RenderShape {
public:
    TreeTrunkShape(double radius_max, double radius_min) : RenderShape(true, "treetrunk.jpeg"), _radius_max(radius_max), _radius_min(radius_min) {}

	template<class Object> void PreCompute(const Object* obj);
    template<class Object> void GetSurface(const Object* obj, Eigen::MatrixXd &vertices, Eigen::MatrixXi &topos) const;
	
protected:
    const double _radius_max;
    const double _radius_min;
};

template<class Object>
void TreeTrunkShape::PreCompute(const Object *obj) {
    const VectorXd& x_curve = obj->_x;
    const int num_points = x_curve.size() / 3;
    _uv_coords.resize(4 * num_points, 2);
    float cur_length = 0;
    float line_length = std::ceil((_radius_max * 4 * sqrt(2))) / 4;
    for (int i = 0, i3 = 0, i4 = 0; i < num_points; i++, i3 += 3, i4 += 4) {
        for (int j = 0; j < 4; j++) {
            _uv_coords.row(i4 + j) << cur_length, line_length * j;
        }
        if (i < num_points - 1) {
            cur_length += (x_curve.segment<3>(i3) - x_curve.segment<3>(i3 + 3)).norm();
        }
    }
}

template<class Object>
void TreeTrunkShape::GetSurface(const Object* obj, Eigen::MatrixXd &vertices, Eigen::MatrixXi &topos) const {
    const VectorXd& x_curve = obj->_x;

    int num_points = x_curve.size() / 3;
    vertices.resize(4 * num_points, 3);
    topos.resize(8 * (num_points - 1), 3);

    Vector3d x_prev = x_curve.segment<3>(0);
    Vector3d x_current = x_curve.segment<3>(3);
    Vector3d t_current = (x_current - x_prev).normalized();

    Vector3d T, U[2];
    T = t_current;
    U[0] = FindPerpendicular(T);
    U[1] = T.cross(U[0]);
    const double delta_radius = (_radius_max - _radius_min) / (num_points - 1);
    double radius[2] = {
        _radius_max, -_radius_max
    };

    Matrix<int, 8, 3> cube_triangulation;
    cube_triangulation <<
        0, 5, 4, 0, 1, 5,
        1, 6, 5, 1, 2, 6,
        2, 7, 6, 2, 3, 7,
        3, 4, 7, 3, 0, 4;

    for (int j = 0; j < 4; j++) {
        vertices.row(j) = x_prev + radius[(j >> 1) & 1] * U[j & 1];
    }

    for (int i = 1, v_index = 4, f_index = 0; i < num_points; i++, v_index += 4, f_index += 8) {
        radius[0] -= delta_radius;
        radius[1] += delta_radius;
        for (int j = 0; j < 4; j++) {
            vertices.row(v_index + j) = x_current + radius[(j >> 1) & 1] * U[j & 1];
        }
        topos.block<8, 3>(f_index, 0) = cube_triangulation;
        cube_triangulation.array() += 4;

        if (i < num_points - 1) {
            Vector3d x_next = x_curve.block<3, 1>(3 * (i + 1), 0);
            Vector3d t_next = (x_next - x_current).normalized();

            Vector3d binormal = t_current.cross(t_next);
            double sin_theta = binormal.norm();
            double cos_theta = t_current.dot(t_next);
            double theta = atan2(sin_theta, cos_theta);

            Matrix3d rotation;
            rotation = AngleAxisd(theta, binormal);
            for (int j : {0, 1}) {
                U[j] = rotation * U[j];
            }

            x_prev = x_current;
            x_current = x_next;
            t_current = t_next;
        }
    }
}
