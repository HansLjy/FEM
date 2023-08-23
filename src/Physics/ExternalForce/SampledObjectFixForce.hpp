#include "ExternalForce.hpp"

template<class Data>
class SampledObjecFixForce : public ExternalForce<Data> {
public:
	explicit SampledObjecFixForce(const json& config) : SampledObjecFixForce(static_cast<double>(config["stiffness"])) {}
	explicit SampledObjecFixForce(double stiffness) : _stiffness(stiffness) {}

	VectorXd GetExternalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const override;
    Vector3d GetTotalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const override;
    Matrix3d GetTotalForceAffineTorque(const Data* data, const Matrix3d& rotation, const Vector3d& position) const override;

protected:
	double _stiffness;
};

template<class Data>
VectorXd SampledObjecFixForce<Data>::GetExternalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const {
	const int num_points = data->_num_points;
	const VectorXd& x_rest = data->_x_rest;
	const VectorXd& x = data->_x;
	VectorXd force(data->_dof);
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		force.segment<3>(i3) = _stiffness * (rotation * x_rest.segment<3>(i3) + position - x.segment<3>(i3));
	}
	return force;
}

template<class Data>
Vector3d SampledObjecFixForce<Data>::GetTotalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const {
	const int num_points = data->_num_points;
	const VectorXd& x_rest = data->_x_rest;
	const VectorXd& x = data->_x;
	Vector3d total_force = Vector3d::Zero();
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		total_force += _stiffness * (rotation * x_rest.segment<3>(i3) + position - x.segment<3>(i3));
	}
	return total_force;
}

template<class Data>
Matrix3d SampledObjecFixForce<Data>::GetTotalForceAffineTorque(const Data* data, const Matrix3d& rotation, const Vector3d& position) const {
	const int num_points = data->_num_points;
	const VectorXd& x_rest = data->_x_rest;
	const VectorXd& x = data->_x;
	Matrix3d total_force_torque = Matrix3d::Zero();
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		Vector3d single_force = _stiffness * (rotation * x_rest.segment<3>(i3) + position - x.segment<3>(i3));
		total_force_torque += single_force * x.segment<3>(i3).transpose();
	}
	return total_force_torque;
}
