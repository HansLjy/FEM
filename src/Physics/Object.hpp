//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_OBJECT_H
#define FEM_OBJECT_H

#include "EigenAll.h"
#include "Pattern.h"
#include "ExternalForce/ExternalForce.hpp"
#include "BlockMatrix.h"

enum class CollisionAssemblerType {
	kNull,
	kIndex,
};

class Object {
public:
	virtual void Initialize() = 0;

	virtual int GetDOF() const = 0;
	virtual void GetCoordinate(Ref<VectorXd> x) const = 0;
	virtual void GetVelocity(Ref<VectorXd> v) const = 0;
	virtual void SetCoordinate(const Ref<const VectorXd>& x) = 0;
	virtual void SetVelocity(const Ref<const VectorXd>& v) = 0;

	/* Physics relevant */
    virtual void GetMass(COO& coo, int x_offset, int y_offset) const = 0;

    virtual double GetPotential(const Ref<const VectorXd>& x) const = 0;
    virtual VectorXd GetPotentialGradient(const Ref<const VectorXd>& x) const = 0;
    virtual void GetPotentialHessian(const Ref<const VectorXd>& x, COO& coo, int x_offset, int y_offset) const = 0;

    virtual void AddExternalForce(const std::string& type, const json& config) = 0;
    virtual VectorXd GetExternalForce() const = 0;

	/* Render shape */
    virtual void GetSurface(MatrixXd &vertices, MatrixXi &topos) const = 0;
	virtual bool IsUsingTexture() const = 0;
	virtual const std::string& GetTexturePath() const = 0;
	virtual void GetUVCoords(MatrixXf& uv_coords) const = 0;

	/* Collision Shape */
	virtual double GetMaxVelocity(const Ref<const VectorXd>& v) const = 0;
    virtual void ComputeCollisionShape(const Ref<const VectorXd>& x) = 0;
	virtual const BlockVector& GetVertexDerivative(int idx) const = 0;
	virtual Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd>& v, int idx) const = 0;
    virtual const MatrixXd& GetCollisionVertices() const = 0;
    virtual const MatrixXi& GetCollisionEdgeTopo() const = 0;
    virtual const MatrixXi& GetCollisionFaceTopo() const = 0;
	virtual int GetCollisionVerticeNumber() const = 0;

	/* Frame relevant */
	virtual Vector3d GetFrameX() const = 0;
	virtual Matrix3d GetFrameRotation() const = 0;

	virtual bool IsDecomposed() const = 0;

	virtual ~Object() = default;
	Object() = default;
	Object(const Object& rhs) = delete;
	Object& operator=(const Object& rhs) = delete;
};

class ProxyObject : public Object {
public:
    virtual double GetTotalMass() const = 0;
	virtual Vector3d GetUnnormalizedMassCenter() const = 0;
	virtual Matrix3d GetInertialTensor() const = 0;
	virtual VectorXd GetExternalForceWithFrame(const Matrix3d& rotation, const Vector3d& position) const = 0;

    virtual Vector3d GetTotalExternalForce(const Matrix3d& rotation, const Vector3d& position) const = 0;
	virtual Matrix3d GetTotalExternalForceTorque(const Matrix3d& rotation, const Vector3d& position) const = 0;
	virtual VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const = 0;
	// The inertial force in **local coordinate**
	virtual VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) const = 0;
};

class ConcreteObject : public ProxyObject{
public:
    ConcreteObject(const VectorXd& x);
    ConcreteObject(const VectorXd& x, const VectorXd& v);

	int GetDOF() const override {return _dof;}
	void GetCoordinate(Ref<VectorXd> x) const override {x = _x;}
	void GetVelocity(Ref<VectorXd> v) const override {v = _v;}
	void SetCoordinate(const Ref<const VectorXd> &x) override {_x = x;}
	void SetVelocity(const Ref<const VectorXd> &v) override {_v = v;}

    /* Physics relevant */
	void GetMass(COO &coo, int x_offset, int y_offset) const override = 0;
	double GetTotalMass() const override = 0;
	Vector3d GetUnnormalizedMassCenter() const override = 0;
	Matrix3d GetInertialTensor() const override = 0;

	double GetPotential(const Ref<const VectorXd> &x) const override = 0;
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override = 0;
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override = 0;

	
    void AddExternalForce(const std::string& type, const json& config) override = 0;
	VectorXd GetExternalForce() const override = 0;
	VectorXd GetExternalForceWithFrame(const Matrix3d &rotation, const Vector3d &position) const override = 0;
	Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const override = 0;
	Matrix3d GetTotalExternalForceTorque(const Matrix3d &rotation, const Vector3d &position) const override = 0;

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override = 0;
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d &affine_velocity, const Matrix3d &affine_acceleration) const override = 0;

	/* Frame relevant */
	Vector3d GetFrameX() const override;
	Matrix3d GetFrameRotation() const override;

	bool IsDecomposed() const override {return false;}

protected:
	int _dof;
    VectorXd _x;
    VectorXd _v;
};

class ObjectFactory {
public:
	using ObjectGenerator = std::function<Object*(const json& config)>;

	static ObjectFactory* Instance() {
		if (_the_factory == nullptr) {
			_the_factory = new ObjectFactory;
		}
		return _the_factory;
	}

	bool Register(const std::string& name, const ObjectGenerator& generator) {
		return _generators.insert(std::make_pair(name, generator)).second;
	}

	Object* GetObject(const std::string& name, const json& config) {
		return _generators[name](config);
	}

protected:
	static ObjectFactory* _the_factory;
	std::map<std::string, ObjectGenerator> _generators;
};

#endif //FEM_OBJECT_H
