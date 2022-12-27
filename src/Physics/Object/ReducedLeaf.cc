//
// Created by hansljy on 11/15/22.
//

#include "ReducedLeaf.h"
#include "Cloth.h"
#include "ReducedBezierSurface.h"
#include "RenderShape/ClothShape/ClothShape.h"
#include "Collision/CollisionShape/ReducedLeafCollisionShape.h"
#include "JsonUtil.h"

ReducedLeaf::ReducedLeaf(const json &config)
    : ReducedLeaf(config["collision-enabled"], config["density"], config["thickness"], config["k-stretch"], config["k-shear"],
                  config["k-bend-max"], config["k-bend-min"],
                  config["u-segments"], config["v-segments"],
                  Json2VecX(config["control-points"])){}

ReducedLeaf::ReducedLeaf(bool collision_enabled, double density, double thickness, double k_stretch, double k_shear, double k_bend_max, double k_bend_min,
                         int u_segments, int v_segments, const VectorXd& control_points)
                         : ReducedObject(
								new ReducedRenderShape,
								collision_enabled ? (CollisionShape*) new ReducedLeafCollisionShape : new NullCollisionShape,
								GenerateX(control_points),
								new Cloth(false, density, thickness, k_stretch, k_shear, k_bend_max, k_bend_min,
									GenerateMaxBendDir(control_points),
									GenerateClothX(control_points, u_segments, v_segments),
									GenerateClothUV(control_points, u_segments, v_segments),
									GenerateClothTopo(u_segments, v_segments)
								),
								GenerateBase(u_segments, v_segments),
								GenerateShift(control_points, u_segments, v_segments)
                           ) {}

VectorXd ReducedLeaf::GenerateX(const Eigen::VectorXd &control_points) {
    VectorXd x(18);
    x << control_points.segment<3>(6), control_points.segment<15>(12);
    return x;
}

VectorXd ReducedLeaf::GenerateClothX(const Eigen::VectorXd &control_points, int u_segments, int v_segments) {
    return ReducedBezierSurface::GenerateX(control_points, u_segments, v_segments);
}

VectorXd ReducedLeaf::GenerateClothUV(const VectorXd &control_points, int u_segments, int v_segments) {
    return ReducedBezierSurface::GenerateUVCoord(control_points, u_segments, v_segments);
}

MatrixXi ReducedLeaf::GenerateClothTopo(int u_segments, int v_segments) {
    return ReducedBezierSurface::GenerateTopo(u_segments, v_segments);
}

SparseMatrixXd ReducedLeaf::GenerateBase(int u_segments, int v_segments) {
    const int num_points = (u_segments + 1) * (v_segments + 1);
    SparseMatrixXd base = ReducedBezierSurface::GetBase(u_segments, v_segments);
    MatrixXd base_dense = base.toDense();
    MatrixXd reduced_base_dense(3 * num_points, 18);
    reduced_base_dense.block(0, 0, 3 * num_points, 3) = base_dense.block(0, 6, 3 * num_points, 3);
    reduced_base_dense.block(0, 3, 3 * num_points, 15) = base_dense.block(0, 12, 3 * num_points, 15);
    return reduced_base_dense.sparseView();
}

VectorXd ReducedLeaf::GenerateShift(const VectorXd& control_points, int u_segments, int v_segments) {
    const int num_points = (u_segments + 1) * (v_segments + 1);
    SparseMatrixXd base = ReducedBezierSurface::GetBase(u_segments, v_segments);
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

