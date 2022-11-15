//
// Created by hansljy on 11/1/22.
//

#include "ReducedBezierSurface.h"
#include "Cloth/Cloth.h"
#include "JsonUtil.h"

DEFINE_CLONE(Object, ReducedBezierSurface)

ReducedBezierSurface::ReducedBezierSurface(const json &config)
    : ReducedBezierSurface(
        Json2VecX(config["control-points"]),
        config["density"], config["k-stretch"], config["k-shear"], config["k-bend-max"], config["k-bend-min"], Json2Vec(config["max-dir"]),
        config["u-segments"], config["v-segments"], config["stretch-u"], config["stretch-v"]
      ) {}

ReducedBezierSurface::ReducedBezierSurface(const Eigen::VectorXd &control_points, double rho, double k_stretch,
                                           double k_shear, double k_bend_max, double k_bend_min,
                                           const Eigen::Vector3d &max_dir, int num_u_segments, int num_v_segments,
                                           double stretch_u, double stretch_v)
    : ReducedObject(control_points,
                    Cloth(rho, k_stretch, k_shear, k_bend_max, k_bend_min,
                          GenerateDir(control_points, max_dir),
                          GenerateX(control_points, num_u_segments, num_v_segments),
                          GenerateUVCoord(control_points, num_u_segments, num_v_segments),
                          GenerateTopo(num_u_segments, num_v_segments),
                          stretch_u, stretch_v
                    ),
                    GetBase(num_u_segments, num_v_segments),
                    GetShift(num_u_segments, num_v_segments)) {}

VectorXd ReducedBezierSurface::GetShift(int num_u_segments, int num_v_segments) {
    VectorXd shift(3 * (num_u_segments + 1) * (num_v_segments + 1));
    shift.setZero();
    return shift;
}

SparseMatrixXd ReducedBezierSurface::GetBase(int num_u_segments, int num_v_segments) {
    const int num_u_control_points = 3;
    const int num_v_control_points = 3;

    const int pascal_coef[] = {1, 2, 1};

    MatrixXd u_coeff(num_u_control_points, num_u_segments + 1);
    MatrixXd v_coeff(num_v_control_points, num_v_segments + 1);

    const double hu = 1.0 / num_u_segments;
    double ih = 0;
    for (int i = 0; i <= num_u_segments; i++, ih += hu) {
        for (int k = 0; k < 3; k++) {
            u_coeff(k, i) = pascal_coef[k] * pow(1 - ih, 2 - k) * pow(ih, k);
        }
    }
    const double hv = 1.0 / num_v_segments;
    double jh = 0;
    for (int i = 0; i <= num_v_segments; i++, jh += hv) {
        for (int k = 0; k < 3; k++) {
            v_coeff(k, i) = pascal_coef[k] * pow(1 - jh, 2 - k) * pow(jh, k);
        }
    }

    auto GetControlPointsIndex = [&, num_u_control_points] (int i, int j) {return num_u_control_points * j + i;};
    auto GetSamplePointsIndex = [&, num_u_segments] (int i, int j) {return (num_u_segments + 1) * j + i;};

    COO coo;
    for (int i = 0; i < num_u_control_points; i++) {
        for (int j = 0; j < num_v_control_points; j++) {
            for (int p = 0; p <= num_u_segments; p++) {
                for (int q = 0; q <= num_v_segments; q++) {
                    const double coef = u_coeff(i, p) * v_coeff(j, q);
                    const int control_point_offset = GetControlPointsIndex(i, j) * 3;
                    const int sample_point_offset = GetSamplePointsIndex(p, q) * 3;
                    for (int k = 0; k < 3; k++) {
                        coo.push_back(Tripletd(sample_point_offset + k, control_point_offset + k, coef));
                    }
                }
            }
        }
    }
    SparseMatrixXd hessian(3 * (num_u_segments + 1) * (num_v_segments + 1), 3 * num_u_control_points * num_v_control_points);
    hessian.setFromTriplets(coo.begin(), coo.end());

    return hessian;
}

VectorXd ReducedBezierSurface::GenerateX(const Eigen::VectorXd &control_points, int num_u_segments, int num_v_segments) {
    SparseMatrixXd base = GetBase(num_u_segments, num_v_segments);
    return base * control_points;
}

MatrixXi ReducedBezierSurface::GenerateTopo(int num_u_segments, int num_v_segments) {
    return Cloth::GenerateTopo(num_u_segments, num_v_segments);
}

VectorXd ReducedBezierSurface::GenerateUVCoord(const Eigen::VectorXd &control_points, int num_u_segments, int num_v_segments) {
    VectorXd x = GenerateX(control_points, num_u_segments, num_v_segments);
    Vector3d lower_left = x.segment<3>(0);
    Vector3d u_dir = (x.segment<3>(3) - lower_left).normalized();
    Vector3d v_dir_unnormalized = x.segment<3>(3 * (num_u_segments + 1)) - lower_left;
    Vector3d v_dir = (v_dir_unnormalized - v_dir_unnormalized.dot(u_dir) * u_dir).normalized();

    const int num_sample_points = (num_u_segments + 1) * (num_v_segments + 1);
    VectorXd uv_coord(2 * num_sample_points);
    for (int i = 0, i2 = 0, i3 = 0; i < num_sample_points; i++, i2 += 2, i3 += 3) {
        Vector3d cur_x = x.segment<3>(i3) - lower_left;
        uv_coord.segment<2>(i2) << cur_x.dot(u_dir), cur_x.dot(v_dir);
    }
    return uv_coord;
}

Vector2d ReducedBezierSurface::GenerateDir(const Eigen::VectorXd &control_points, const Eigen::Vector3d &max_dir) {
    Vector3d lower_left = control_points.segment<3>(0);
    Vector3d u_dir = (control_points.segment<3>(6) - lower_left).normalized();
    Vector3d v_dir = (control_points.segment<3>(18) - lower_left).normalized();
    return (Vector2d() << u_dir.dot(max_dir), v_dir.dot(max_dir)).finished();

}