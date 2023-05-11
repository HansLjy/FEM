#pragma once

#include "Object.hpp"

class SampledObject : public ConcreteObject {
public:
	SampledObject(const VectorXd& x, const VectorXd& mass, int dimension, const MatrixXi& topo);
	SampledObject(const VectorXd& x, const VectorXd& v, const VectorXd& mass, int dimension, const MatrixXi& topo);
	
	void GetMass(COO &coo, int x_offset, int y_offset) const override;
	double GetTotalMass() const override;
	Vector3d GetUnnormalizedMassCenter() const override;
	Matrix3d GetInertialTensor() const override;

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override;
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d &affine_velocity, const Matrix3d &affine_acceleration) const override;

protected:
	int _num_points;
	MatrixXi _edge_topo;
	MatrixXi _face_topo;
	MatrixXi _tet_topo;

	double _total_mass;
	VectorXd _mass;
};
