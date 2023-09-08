//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_OBJECT_H
#define FEM_OBJECT_H

#include "EigenAll.h"
#include "Pattern.h"
#include "ExternalForce/ExternalForceContainer.hpp"
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

	virtual Vector3d GetFrameX() const {
		return Vector3d::Zero();
	}
	virtual Matrix3d GetFrameRotation() const {
		return Matrix3d::Identity();
	}
	virtual bool IsDecomposed() const {
		return false;
	}

	/* Mass */
    virtual void GetMass(COO& coo, int x_offset, int y_offset) const = 0;
	virtual double GetTotalMass() const = 0;
	virtual Vector3d GetUnnormalizedMassCenter() const = 0;
	virtual Matrix3d GetInertialTensor() const = 0;
	virtual VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const = 0;
	// The inertial force in **local coordinate**
	virtual VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) const = 0;

	/* Internal Energy */
    virtual double GetPotential(const Ref<const VectorXd>& x) const = 0;
    virtual VectorXd GetPotentialGradient(const Ref<const VectorXd>& x) const = 0;
    virtual void GetPotentialHessian(const Ref<const VectorXd>& x, COO& coo, int x_offset, int y_offset) const = 0;

	/* External Force */

    virtual void AddExternalForce(const std::string& type, const json& config) = 0;
    virtual VectorXd GetExternalForce() const = 0;

	virtual VectorXd GetExternalForceWithFrame(const Matrix3d &rotation, const Vector3d &position) const = 0;
	virtual Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const = 0;
	virtual Matrix3d GetTotalExternalForceTorque(const Matrix3d &rotation, const Vector3d &position) const = 0;

	/* Render Shape */
	virtual void GetRenderVertices(MatrixXd& vertices) const = 0;
	virtual bool IsRenderTopoUpdated() = 0;
	virtual void GetRenderTopos(MatrixXi& topos) const = 0;
	virtual bool HasOuterFrame() const = 0;
	virtual void GetFrameVertices(MatrixXd& vertices) const = 0;
	virtual bool IsFrameTopoUpdated() = 0;
	virtual void GetFrameTopo(MatrixXi& topo) const = 0;
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

	virtual ~Object() = default;
};

template<class Data, class DataForExternalForce, class Coordinate, class MassModel, class EnergyModel, class Render, class Collision>
class ConcreteObject : public Object, public Data, public EnergyModel, public Render, public Collision, public ExternalForceContainer<DataForExternalForce> {
public:
	ConcreteObject(const json& config) : Data(config), EnergyModel(config["energy-model"]), Render(config["render"]), Collision(config["collision"]) {}
	ConcreteObject(const Data&& data, const EnergyModel&& energy_model, const Render&& render, const Collision&& collision) : Data(data), EnergyModel(energy_model), Render(render), Collision(collision) {}

	void Initialize() override {
		EnergyModel::Initialize(this);
		Render::Initialize(this);
		Collision::Initialize(this);
	}

	int GetDOF() const override {return Coordinate::GetDOF(this);}
	void GetCoordinate(Ref<VectorXd> x) const override {Coordinate::GetCoordinate(this, x);}
	void GetVelocity(Ref<VectorXd> v) const override {Coordinate::GetVelocity(this, v);}
	void SetCoordinate(const Ref<const VectorXd> &x) override {Coordinate::SetCoordinate(this, x);}
	void SetVelocity(const Ref<const VectorXd> &v) override {Coordinate::SetVelocity(this, v);}

	void GetMass(COO &coo, int x_offset, int y_offset) const override {MassModel::GetMass(this, coo, x_offset, y_offset);}
	double GetTotalMass() const override {return MassModel::GetTotalMass(this);}
	Vector3d GetUnnormalizedMassCenter() const override {return MassModel::GetUnnormalizedMassCenter(this);}
	Matrix3d GetInertialTensor() const override {return MassModel::GetInertialTensor(this);}
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override {return MassModel::GetInertialForce(this, v, a, omega, alpha, rotation);}
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d &affine_velocity, const Matrix3d &affine_acceleration) const override {return MassModel::GetInertialForce(this, v, a, affine, affine_velocity, affine_acceleration);}

	double GetPotential(const Ref<const VectorXd> &x) const override {return EnergyModel::GetPotential(this, x);}
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override {return EnergyModel::GetPotentialGradient(this, x);}
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override {EnergyModel::GetPotentialHessian(this, x, coo, x_offset, y_offset);}

	void GetRenderVertices(MatrixXd &vertices) const override {Render::GetRenderVertices(this, vertices);}
	void GetRenderTopos(MatrixXi &topos) const override {Render::GetRenderTopos(this, topos);}
	bool HasOuterFrame() const override {return Render::HasOuterFrame(this);}
	void GetFrameVertices(MatrixXd &vertices) const override {Render::GetFrameVertices(this, vertices);}
	void GetFrameTopo(MatrixXi &topo) const override {Render::GetFrameTopo(this, topo);};
	bool IsUsingTexture() const override {return Render::IsUsingTexture(this);}
	const std::string & GetTexturePath() const override {return Render::GetTexturePath(this);}
	void GetUVCoords(MatrixXf &uv_coords) const override {Render::GetUVCoords(this, uv_coords);}
	bool IsRenderTopoUpdated() override {return Render::IsTopoUpdated(this);}
	bool IsFrameTopoUpdated() override {return Render::IsBBTopoUpdated(this);}

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override {return Collision::GetMaxVelocity(this, v);}
	void ComputeCollisionShape(const Ref<const VectorXd> &x) override {Collision::ComputeCollisionShape(this, x);}
	const BlockVector & GetVertexDerivative(int idx) const override {return Collision::GetVertexDerivative(this, idx);}
	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const override {return Collision::GetCollisionVertexVelocity(this, v, idx);}
	const MatrixXd & GetCollisionVertices() const override {return Collision::GetCollisionVertices(this);}
	const MatrixXi & GetCollisionEdgeTopo() const override {return Collision::GetCollisionEdgeTopo(this);}
	const MatrixXi & GetCollisionFaceTopo() const override {return Collision::GetCollisionFaceTopo(this);}
	int GetCollisionVerticeNumber() const override {return Collision::GetCollisionVerticeNumber(this);}

    void AddExternalForce(const std::string& type, const json& config) override {ExternalForceContainer<DataForExternalForce>::AddExternalForce(type, config);}
	VectorXd GetExternalForce() const override {return ExternalForceContainer<DataForExternalForce>::GetExternalForce(this);}

	VectorXd GetExternalForceWithFrame(const Matrix3d &rotation, const Vector3d &position) const override {return ExternalForceContainer<DataForExternalForce>::GetExternalForceWithFrame(this, rotation, position);}
	Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const override {return ExternalForceContainer<DataForExternalForce>::GetTotalExternalForce(this, rotation, position);}
	Matrix3d GetTotalExternalForceTorque(const Matrix3d &rotation, const Vector3d &position) const override {return ExternalForceContainer<DataForExternalForce>::GetTotalExternalForceTorque(this, rotation, position);}
};

#endif //FEM_OBJECT_H
