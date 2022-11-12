//
// Created by hansljy on 11/11/22.
//

#include "ReducedTreeTrunk.h"
#include "TreeTrunk.h"
#include "JsonUtil.h"
#include "ReducedObject/ReducedBezierCurve.h"

DEFINE_CLONE(Object, ReducedTreeTrunk)

ReducedTreeTrunk::ReducedTreeTrunk(const json &config)
    : ReducedTreeTrunk(
        config["segments"], config["density"], config["alpha-max"], config["alpha-min"],
        config["radius-max"], config["radius-min"], config["k"],
        Json2Vec(config["root"]), Json2VecX(config["control-points"])
) {}

ReducedTreeTrunk::ReducedTreeTrunk(int num_segments, double rho, double alpha_max, double alpha_min, double radius_max,
                                   double radius_min, double k, const Vector3d &root, const VectorXd &control_points)
   : ReducedObject(control_points.segment<9>(3),
                   TreeTrunk(rho, alpha_max, alpha_min, radius_max, radius_min, k,
                             GenerateX(num_segments, control_points), root),
                   GenerateBase(num_segments),
                   GenerateShift(num_segments, control_points.segment<3>(0))),
     _x_root(root) {}


VectorXd ReducedTreeTrunk::GenerateX(int num_segments, const VectorXd &control_points) {
    return ReducedBezierCurve::GenerateSamplePoints(num_segments, control_points);
}

SparseMatrixXd ReducedTreeTrunk::GenerateBase(int num_segments) {
    return ReducedBezierCurve::GenerateBase(num_segments).block(0, 3, 3 * (num_segments + 1), 9);
}

VectorXd ReducedTreeTrunk::GenerateShift(int num_segments, const Eigen::Vector3d &first_control_point) {
    return ReducedBezierCurve::GenerateBase(num_segments).block(0, 0, 3 * (num_segments + 1), 3) * first_control_point;
}
