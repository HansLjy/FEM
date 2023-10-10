#include "CurveData.hpp"
#include "ReducedDataUtils.hpp"
#include "ExternalForce/ExternalForce.hpp"

template<>
Factory<ExternalForce<CurveData>>* Factory<ExternalForce<CurveData>>::_the_factory = nullptr;

CurveData::CurveData(const json& config)
: CurveData (config["density"], config["alpha-max"], config["alpha-min"], Json2Vec(config["start"]), Json2Vec(config["end"]), config["segments"]) {}

CurveData::CurveData(double rho, double alpha_max, double alpha_min, const Vector3d &start, const Vector3d &end, int num_segments)
: CurveData(rho, alpha_max, alpha_min, GetX(start, end, num_segments)) {}

CurveData::CurveData(double rho, double alpha_max, double alpha_min, const VectorXd &x)
    : SampledObjectData(x, GenerateMass(rho, x), 1, GenerateTopo(x.size() / 3)),
      _stiffness(100 * alpha_min), _curve_num_points(x.size() / 3) {
    _alpha.resize(_curve_num_points - 1);
    if (_curve_num_points > 2) {
        const double delta_alpha = (alpha_min - alpha_max) / (_curve_num_points - 3);
        double current_alpha = alpha_max;
        for (int i = 1; i < _curve_num_points - 1; i++) {
            _alpha(i) = current_alpha;
            current_alpha += delta_alpha;
        }
    }

    _x_rest = x;
    _rest_length.resize(_curve_num_points - 1);
    for (int i = 0, j = 0; i < _curve_num_points - 1; i++, j += 3) {
        _rest_length(i) = (_x_rest.block<3, 1>(j + 3, 0) - _x_rest.block<3, 1>(j, 0)).norm();
    }
    _voronoi_length.resize(_curve_num_points);
    for (int i = 1; i < _curve_num_points - 1; i++) {
        _voronoi_length(i) = (_rest_length(i - 1) + _rest_length(i)) / 2;
    }
    _voronoi_length(0) = _rest_length(0) / 2;
    _voronoi_length(_curve_num_points - 1) = _rest_length(_curve_num_points - 2) / 2;
}

VectorXd CurveData::GetX(const Vector3d &start, const Vector3d &end, int num_segments){
    VectorXd x(3 * (num_segments + 1));
    Vector3d delta = (end - start) / num_segments;
    Vector3d current = start;
    for (int i = 0, j = 0; i <= num_segments; i++, j += 3, current += delta) {
        x.segment<3>(j) = current;
    }
    return x;
}

VectorXd CurveData::GenerateMass(double rho, const Eigen::VectorXd &x) {
    int num_points = x.size() / 3;
    VectorXd mass(num_points);
    mass.setZero();
    for (int i = 0, j = 0; i < num_points - 1; i++, j += 3) {
        Vector3d e = x.segment<3>(j + 3) - x.segment<3>(j);
        mass(i) += e.norm() * rho / 2;
        mass(i + 1) += e.norm() * rho / 2;
    }
    return mass;
}

MatrixXi CurveData::GenerateTopo(int n) {
	MatrixXi edge_topo(n - 1, 2);
	for (int i = 0; i < n - 1; i++) {
		edge_topo.row(i) << i, i + 1;
	}
	return edge_topo;
}

BezierCurveData::BezierCurveData(const json& config)
: BezierCurveData(config["segments"], config["density"], config["alpha-max"], config["alpha-min"],	Json2VecX(config["control-points"])) {}

BezierCurveData::BezierCurveData(int num_segments, double rho, double alpha_max, double alpha_min, const VectorXd &control_points)
: ReducedObjectData<CurveData>(
	control_points,
	new CurveData(
		rho, alpha_max, alpha_min, ReducedDataUtils::BezierCurve::GenerateSamplePoints(control_points, num_segments)),
		ReducedDataUtils::BezierCurve::GenerateBase(num_segments),
		ReducedDataUtils::BezierCurve::GenerateShift(num_segments)
	) {}
