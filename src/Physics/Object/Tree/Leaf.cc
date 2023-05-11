//
// Created by hansljy on 11/15/22.
//

#include "Object/Cloth.hpp"
#include "Object/ReducedObjectUtils.hpp"
#include "Leaf.hpp"
#include "JsonUtil.h"

namespace {
    const bool leaf_registered = ObjectFactory::Instance()->Register("leaf", [](const json& config) {return new ReducedLeaf(config);});
}


ReducedLeaf::ReducedLeaf(const json &config)
    : ReducedLeaf(config["density"], config["thickness"], config["k-stretch"], config["k-shear"],
                  config["k-bend-max"], config["k-bend-min"],
                  config["u-segments"], config["v-segments"],
                  Json2VecX(config["control-points"])){}

ReducedLeaf::ReducedLeaf(double density, double thickness, double k_stretch, double k_shear, double k_bend_max, double k_bend_min,
                         int num_u_segments, int num_v_segments, const VectorXd& control_points)
                         : ReducedObject(
								GenerateX(control_points),
								new Cloth(density, thickness, k_stretch, k_shear, k_bend_max, k_bend_min,
									GenerateMaxBendDir(control_points),
									ReducedObjectUtils::BezierSurface::GenerateSamplePoints(control_points, num_u_segments, num_v_segments),
									ReducedObjectUtils::BezierSurface::GenerateUVCoord(control_points, num_u_segments, num_v_segments),
									Cloth::GenerateTopo(num_u_segments, num_v_segments)
								),
								GenerateBase(num_u_segments, num_v_segments),
								GenerateShift(control_points, num_u_segments, num_v_segments)
                           ) {}

VectorXd ReducedLeaf::GenerateX(const Eigen::VectorXd &control_points) {
    VectorXd x(18);
    x << control_points.segment<3>(6), control_points.segment<15>(12);
    return x;
}


SparseMatrixXd ReducedLeaf::GenerateBase(int num_u_segments, int num_v_segments) {
    const int num_points = (num_u_segments + 1) * (num_v_segments + 1);
    SparseMatrixXd base = ReducedObjectUtils::BezierSurface::GenerateBase(num_u_segments, num_v_segments);
    MatrixXd base_dense = base.toDense();
    MatrixXd reduced_base_dense(3 * num_points, 18);
    reduced_base_dense.block(0, 0, 3 * num_points, 3) = base_dense.block(0, 6, 3 * num_points, 3);
    reduced_base_dense.block(0, 3, 3 * num_points, 15) = base_dense.block(0, 12, 3 * num_points, 15);
    return reduced_base_dense.sparseView();
}

VectorXd ReducedLeaf::GenerateShift(const VectorXd& control_points, int num_u_segments, int num_v_segments) {
    const int num_points = (num_u_segments + 1) * (num_v_segments + 1);
    SparseMatrixXd base = ReducedObjectUtils::BezierSurface::GenerateBase(num_u_segments, num_v_segments);
    return base.block(0, 0, 3 * num_points, 6) * control_points.segment<6>(0)
           + base.block(0, 9, 3 * num_points, 3) * control_points.segment<3>(9);
}

Vector2d ReducedLeaf::GenerateMaxBendDir(const VectorXd &control_points) {
    Vector3d u_dir = (control_points.segment<3>(3) - control_points.segment<3>(0)).normalized();
    Vector3d v_dir = (control_points.segment<3>(9) - control_points.segment<3>(0)).normalized();
    Vector3d max_dir = (control_points.segment<3>(24) - control_points.segment<3>(0)).normalized();
    const double u = max_dir.dot(u_dir);
    return (Vector2d() << u, sqrt(1 - u * u)).finished().normalized();
}

