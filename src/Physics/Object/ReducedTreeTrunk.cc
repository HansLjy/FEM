//
// Created by hansljy on 11/11/22.
//

#include "ReducedTreeTrunk.h"
#include "TreeTrunk.h"
#include "RenderShape/TreeShape/TreeTrunkShape.h"
#include "JsonUtil.h"
#include "ReducedBezierCurve.h"
#include "Collision/CollisionShape/ReducedTreeTrunkCollisionShape.h"

ReducedTreeTrunk::ReducedTreeTrunk(const json &config)
    : ReducedTreeTrunk(
		config["collision-enabled"],
        config["segments"], config["density"], config["youngs-module"],
        config["radius-max"], config["radius-min"],
        Json2Vec(config["root"]), Json2VecX(config["control-points"])
) {}

ReducedTreeTrunk::ReducedTreeTrunk(bool collision_enabled, int num_segments, double rho, double youngs_module, double radius_max,
                                   double radius_min,
                                   const Vector3d &root, const VectorXd &control_points)
   : ReducedObject(new ReducedRenderShape,
				   collision_enabled ? (CollisionShape*) new ReducedTreeTrunkCollisionShape : new NullCollisionShape,
				   control_points.segment<9>(3),
                   new TreeTrunk(false, rho, youngs_module, radius_max, radius_min,
                             GenerateX(num_segments, control_points), root),
                   GenerateBase(num_segments),
                   GenerateShift(num_segments, control_points.segment<3>(0))),
     _x_root(root), _fixed_point(control_points.segment<3>(0)) {}


VectorXd ReducedTreeTrunk::GenerateX(int num_segments, const VectorXd &control_points) {
    return ReducedBezierCurve::GenerateSamplePoints(num_segments, control_points);
}

SparseMatrixXd ReducedTreeTrunk::GenerateBase(int num_segments) {
    return ReducedBezierCurve::GenerateBase(num_segments).block(0, 3, 3 * (num_segments + 1), 9);
}

VectorXd ReducedTreeTrunk::GenerateShift(int num_segments, const Eigen::Vector3d &first_control_point) {
    return ReducedBezierCurve::GenerateBase(num_segments).block(0, 0, 3 * (num_segments + 1), 3) * first_control_point;
}

MatrixXd ReducedTreeTrunk::GetChildProjection(double distance) const {
	const int num_segments = _proxy->GetDOF() / 3 - 1;
    const double delta_t = 1.0 / num_segments;
    const int segment_id = floor(distance / delta_t);
    const double coef = (distance - delta_t * segment_id) / delta_t;
    const auto& project_prev = _base.block(3 * segment_id, 0, 3, 9);
    const auto& project_next = _base.block(3 * (segment_id + 1), 0, 3, 9);
    return project_prev * (1 - coef) + project_next * coef;
}
