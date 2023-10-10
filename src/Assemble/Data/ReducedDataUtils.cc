#include "ReducedDataUtils.hpp"

VectorXd ReducedDataUtils::BezierCurve::GenerateSamplePoints(const VectorXd &control_points, int num_segments) {
    VectorXd sample_points((num_segments + 1) * 3);
    double h = 1.0 / num_segments;
    for (int i = 0, j = 0; i <= num_segments; i++, j += 3) {
        double t = i * h;
        sample_points.segment<3>(j) =
            (1 - t) * (1 - t) * (1 - t) * control_points.segment<3>(0) +
            3 * (1 - t) * (1 - t) * t * control_points.segment<3>(3) +
            3 * (1 - t) * t * t * control_points.segment<3>(6) +
            t * t * t * control_points.segment<3>(9);
    }
    return sample_points;
}

SparseMatrixXd ReducedDataUtils::BezierCurve::GenerateBase(int num_segments) {
    SparseMatrixXd base((num_segments + 1) * 3, 12);
    COO coo;
    const double h = 1.0 / num_segments;
    for (int i = 0, ii = 0; i <= num_segments; i++, ii += 3) {
        double t = i * h;
        double B[4] = {
                (1 - t) * (1 - t) * (1 - t),
                3 * (1 - t) * (1 - t) * t,
                3 * (1 - t) * t * t,
                t * t * t
        };
        for (int j = 0, jj = 0; j < 4; j++, jj += 3) {
            for (int k = 0; k < 3; k++) {
                coo.push_back(Tripletd(ii + k, jj + k, B[j]));
            }
        }
    }
    base.setFromTriplets(coo.begin(), coo.end());
    return base;
}

VectorXd ReducedDataUtils::BezierCurve::GenerateShift(int num_segments) {
    return VectorXd::Zero(3 * (num_segments + 1));
}

VectorXd ReducedDataUtils::BezierSurface::GenerateSamplePoints(const VectorXd &control_points, int num_u_segments, int num_v_segments){
    SparseMatrixXd base = GenerateBase(num_u_segments, num_v_segments);
    return base * control_points;
}

MatrixXd ReducedDataUtils::BezierSurface::GenerateUVCoord(const VectorXd &control_points, int num_u_segments, int num_v_segments) {
    VectorXd x = GenerateSamplePoints(control_points, num_u_segments, num_v_segments);
    Vector3d lower_left = x.segment<3>(0);
    Vector3d u_dir = (x.segment<3>(3) - lower_left).normalized();
    Vector3d v_dir_unnormalized = x.segment<3>(3 * (num_u_segments + 1)) - lower_left;
    Vector3d v_dir = (v_dir_unnormalized - v_dir_unnormalized.dot(u_dir) * u_dir).normalized();

    const int num_sample_points = (num_u_segments + 1) * (num_v_segments + 1);
    VectorXd uv_coord(num_sample_points, 2);
    for (int i = 0, i3 = 0; i < num_sample_points; i++, i3 += 3) {
        Vector3d cur_x = x.segment<3>(i3) - lower_left;
        uv_coord.row(i) << cur_x.dot(u_dir), cur_x.dot(v_dir);
    }
    return uv_coord;
}

Vector2d ReducedDataUtils::BezierSurface::GenerateUVDir(const VectorXd &control_points, const Vector3d &dir) {
    Vector3d lower_left = control_points.segment<3>(0);
    Vector3d u_dir = (control_points.segment<3>(6) - lower_left).normalized();
    Vector3d v_dir = (control_points.segment<3>(18) - lower_left).normalized();
    return (Vector2d() << u_dir.dot(dir), v_dir.dot(dir)).finished();
}

SparseMatrixXd ReducedDataUtils::BezierSurface::GenerateBase(int num_u_segments, int num_v_segments) {
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

VectorXd ReducedDataUtils::BezierSurface::GenerateShift(int num_u_segments, int num_v_segments) {
    return VectorXd::Zero(3 * (num_u_segments + 1) * (num_v_segments + 1));
}