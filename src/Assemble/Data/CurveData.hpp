#pragma once

#include "SampledData.hpp"
#include "JsonUtil.h"

struct CurveData : public SampledObjectData {
	explicit CurveData(const json& config);
	CurveData(double rho, double alpha_max, double alpha_min, const Vector3d &start, const Vector3d &end, int num_segments);
    CurveData(double rho, double alpha_max, double alpha_min, const VectorXd &x);
	

    static VectorXd GetX(const Vector3d& start, const Vector3d& end, int num_segments);
    static VectorXd GenerateMass(double rho, const VectorXd &x);
	static MatrixXi GenerateTopo(int n);

    const double _stiffness;	// stiffness of the curve
    int _curve_num_points;      // number of sampled points (end points included)
    VectorXd _alpha;
    VectorXd _x_rest;           // rest shape of the curve
    VectorXd _rest_length;      // length of every edge in the rest shape
    VectorXd _voronoi_length;   // length under the government of one point

};

struct BezierCurveData : public ReducedObjectData<CurveData> {
	explicit BezierCurveData(const json& config);
    BezierCurveData(int num_segments, double rho, double alpha_max, double alpha_min, const VectorXd &control_points);
};