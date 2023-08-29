#pragma once
#include "ExternalForce.hpp"
#include "JsonUtil.h"

template<class Data>
class SampledObjecFixtureForce : public ExternalForce<Data> {
public:
	explicit SampledObjecFixtureForce(const json& config) : SampledObjecFixtureForce(
		config["stiffness"],
		Json2VecX(config["fix-points"]),
		Json2VecX<int>(config["fix-ids"])
	) {}
	explicit SampledObjecFixtureForce(double stiffness, const VectorXd& fix_points, const VectorXi& fix_id) : _stiffness(stiffness), _fix_points(fix_points), _fix_id(fix_id) {}

	VectorXd GetExternalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const override;
    Vector3d GetTotalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const override;
    Matrix3d GetTotalForceAffineTorque(const Data* data, const Matrix3d& rotation, const Vector3d& position) const override;

protected:
	double _stiffness;
	VectorXd _fix_points;
	VectorXi _fix_id;
};

template<class Data>
VectorXd SampledObjecFixtureForce<Data>::GetExternalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const {
	const int num_fix_points = _fix_id.size();
	const VectorXd& x = data->_x;
	VectorXd force = VectorXd::Zero(data->_dof);
	for (int i = 0, i3 = 0; i < num_fix_points; i++, i3 += 3) {
		force.segment<3>(_fix_id(i) * 3) += _stiffness * (rotation * _fix_points.segment<3>(i3) + position - x.segment<3>(_fix_id(i) * 3));
	}
	return force;
}

template<class Data>
Vector3d SampledObjecFixtureForce<Data>::GetTotalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const {
	throw std::logic_error("Unimplemented method");
	// const int num_points = data->_num_points;
	// const VectorXd& x_rest = data->_x_rest;
	// const VectorXd& x = data->_x;
	// Vector3d total_force = Vector3d::Zero();
	// for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
	// 	total_force += _stiffness * (rotation * x_rest.segment<3>(i3) + position - x.segment<3>(i3));
	// }
	// return total_force;
}

template<class Data>
Matrix3d SampledObjecFixtureForce<Data>::GetTotalForceAffineTorque(const Data* data, const Matrix3d& rotation, const Vector3d& position) const {
	throw std::logic_error("Unimplemented method");
	// const int num_points = data->_num_points;
	// const VectorXd& x_rest = data->_x_rest;
	// const VectorXd& x = data->_x;
	// Matrix3d total_force_torque = Matrix3d::Zero();
	// for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
	// 	Vector3d single_force = _stiffness * (rotation * x_rest.segment<3>(i3) + position - x.segment<3>(i3));
	// 	total_force_torque += single_force * x.segment<3>(i3).transpose();
	// }
	// return total_force_torque;
}