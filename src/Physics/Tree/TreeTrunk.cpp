//
// Created by hansljy on 11/10/22.
//

#include "TreeTrunk.h"
#include "JsonUtil.h"

DEFINE_CLONE(Object, TreeTrunk)

TreeTrunk::TreeTrunk(const json &json) : TreeTrunk(
    json["segments"], json["density"], json["alpha-max"], json["alpha-min"],
    Json2VecX(json["control-points"]), Json2Vec(json["x-root"])
){}

TreeTrunk::TreeTrunk(int num_segments, double rho, double alpha_max, double alpha_min,
                     const Eigen::VectorXd &control_points, const Eigen::Vector3d &x_root)
                     : ReducedBezierCurve(num_segments, rho, alpha_max, alpha_min, control_points), _x_root(x_root) {}