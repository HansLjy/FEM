//
// Created by hansljy on 10/18/22.
//

#include "RenderShape/RenderShape.h"
#include "Collision/CollisionShape/ReducedBezierCurveCollisionShape.h"
#include "ReducedBezierCurve.h"
#include "JsonUtil.h"
#include "Curve.h"
#include <vector>

ReducedBezierCurve::ReducedBezierCurve(const nlohmann::json &config) :
        ReducedBezierCurve(
			config["collision-enabled"], config["segments"], config["density"], config["alpha-max"], config["alpha-min"],
			Json2VecX(config["control-points"])
        ) {}

VectorXd
ReducedBezierCurve::GenerateSamplePoints(int num_segments, const VectorXd &control_points) {
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

#include "Curve.h"

ReducedBezierCurve::ReducedBezierCurve(bool collision_enabled, int num_segments, double rho, double alpha_max, double alpha_min,
                                       const VectorXd &control_points)
        : ReducedObject(new ReducedRenderShape,
						collision_enabled ? (CollisionShape*) new ReducedBezierCurveCollisionShape : new NullCollisionShape,
						control_points,
                        new Curve(false, rho, alpha_max, alpha_min, GenerateSamplePoints(num_segments, control_points)),
                        GenerateBase(num_segments),
                        GenerateShift(num_segments)) {}

VectorXd ReducedBezierCurve::GenerateShift(int num_segments) {
    VectorXd shift(3 * (num_segments + 1));
    shift.setZero();
    return shift;
}

SparseMatrixXd ReducedBezierCurve::GenerateBase(int num_segments) {
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