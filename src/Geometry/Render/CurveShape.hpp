//
// Created by hansljy on 10/13/22.
//

#pragma once

#include "RenderShape.hpp"

class CurveRenderShape : public RenderShape {
public:
    explicit CurveRenderShape(double radius) : RenderShape(false, false, ""), _radius(radius) {}
    template<class Data> void GetRenderVertices(const Data* data, Ref<MatrixXd> vertices) const;
    template<class Data> void GetRenderTopos(const Data* data, Ref<MatrixXi> topos) const;

public:
    const double _radius;
};

template<class Data>
void CurveRenderShape::GetRenderTopos(const Data *data, Ref<MatrixXi> topos) const {
    int num_points = data->_x.size() / 3;
    topos.resize(12 * (num_points - 1), 3);

    Matrix<int, 12, 3> cube_triangulation;
    cube_triangulation << 0, 2, 1, 0, 3, 2,
                          4, 5, 6, 4, 6, 7,
                          0, 5, 4, 0, 1, 5,
                          1, 6, 5, 1, 2, 6,
                          2, 7, 6, 2, 3, 7,
                          3, 4, 7, 3, 0, 4;

    for (int i = 1, f_index = 0; i < num_points; i++, f_index += 12) {
        topos.block<12, 3>(f_index, 0) = cube_triangulation;
        cube_triangulation.array() += 8;
	}
}

template<class Data>
void CurveRenderShape::GetRenderVertices(const Data* data, Ref<MatrixXd> vertices) const {
    const VectorXd& x_curve = data->_x;
    int num_points = x_curve.size() / 3;
    vertices.resize(8 * (num_points - 1), 3);

    Vector3d x_prev = x_curve.block<3, 1>(0, 0);
    Vector3d x_current = x_curve.block<3, 1>(3, 0);
    Vector3d t_current = (x_curve.block<3, 1>(3, 0) - x_prev).normalized();

    Vector3d T, U[2];
    T = t_current;
    U[0] = FindPerpendicular(T);
    U[1] = T.cross(U[0]);
    const double coef [] = {_radius, -_radius};

    for (int i = 1, v_index = 0, f_index = 0; i < num_points; i++, v_index += 8, f_index += 12) {
        for (int j = 0; j < 8; j++) {
            vertices.row(v_index + j) = ((j & 4) ? x_current : x_prev) + coef[(j >> 1) & 1] * U[j & 1];
        }

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