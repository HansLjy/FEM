#include "Object.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

class FixedObject : public ConcreteObject, public FixedRenderShape, public FixedCollisionShape {
public:
	FixedObject(const json& config) : ConcreteObject(VectorXd(0)), FixedRenderShape(config["render-object"]), FixedCollisionShape(config["collision-object"]) {}

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override {return 0;}
	void GetMass(COO &coo, int x_offset, int y_offset) const override {}
	
	double GetTotalMass() const override {return 0;}
	Vector3d GetUnnormalizedMassCenter() const override {return Vector3d::Zero();}
	Matrix3d GetInertialTensor() const override {return Matrix3d::Zero();}

	double GetPotential(const Ref<const VectorXd> &x) const override {return 0;}
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override {return VectorXd(0);}
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override {}

	Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const override {return Vector3d::Zero();}
	Matrix3d GetTotalExternalForceTorque(const Matrix3d &rotation, const Vector3d &position) const override {return Matrix3d::Zero();}

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override {return VectorXd(0);}
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d &affine_velocity, const Matrix3d &affine_acceleration) const override {return VectorXd(0);}
};
