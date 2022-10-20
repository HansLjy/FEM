//
// Created by hansljy on 10/18/22.
//

#include "ReducedBezierCurve.h"
#include "JsonUtil.h"
#include <vector>

DEFINE_CLONE(Object, ReducedBezierCurve)

ReducedBezierCurve::ReducedBezierCurve(const nlohmann::json &config) :
        ReducedBezierCurve(
                config["segments"], config["mass"], config["alpha-max"], config["alpha-min"],
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

#include "Curve/ExtensibleCurve.h"

ReducedBezierCurve::ReducedBezierCurve(int num_segments, double mass, double alpha_max, double alpha_min,
                                       const VectorXd &control_points)
        : ReducedObject(control_points, ExtensibleCurve(mass, alpha_max, alpha_min, GenerateSamplePoints(num_segments, control_points)), GenerateBase(num_segments)) {}

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