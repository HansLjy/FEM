#include "CurveData.hpp"
#include "ExternalForce/ExternalForce.hpp"
#include "InitUtils.hpp"

template<>
Factory<ExternalForce<CurveData>>* Factory<ExternalForce<CurveData>>::_the_factory = nullptr;


CurveData::CurveData(
    const VectorXd& x,
    const VectorXd& mass,
    const VectorXd& alpha,
    double stiffness) :
    SampledObjectData(GetSampledObjectData(
		x, mass,
		InitializationUtils::Curve::GenerateCurveEdgeTopo(x.size() / 3),
		1
	)),
    _stiffness(stiffness) {

    _x_rest = x;
    _rest_length.resize(_num_points - 1);
    for (int i = 0, j = 0; i < _num_points - 1; i++, j += 3) {
        _rest_length(i) = (
            _x_rest.block<3, 1>(j + 3, 0) -
            _x_rest.block<3, 1>(j, 0)
        ).norm();
    }
    _voronoi_length.resize(_num_points);
    for (int i = 1; i < _num_points - 1; i++) {
        _voronoi_length(i) = (_rest_length(i - 1) + _rest_length(i)) / 2;
    }
    _voronoi_length(0) = _rest_length(0) / 2;
    _voronoi_length(_num_points - 1) = _rest_length(_num_points - 2) / 2;
}