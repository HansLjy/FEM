#pragma once

#include "EigenAll.h"
#include "Collision/CollisionShape/CollisionShape.hpp"
#include "Collision/CollisionInfo.hpp"

class CollisionInterface {
public:
	template<class T>
	CollisionInterface(T* obj) : _concept(new Model<T>(obj)) {}

	const BlockVector& GetCollisionVertexDerivative(int idx) const {
		return _concept->GetCollisionVertexDerivative(idx);
	}
	Vector3d GetCollisionVertexVelocity(int idx) const {
		return _concept->GetCollisionVertexVelocity(idx);
	}
	const MatrixXd& GetCollisionVertices() const {
		return _concept->GetCollisionVertices();
	}
	const MatrixXi& GetCollisionEdgeTopo() const {
		return _concept->GetCollisionEdgeTopo();
	}
	const MatrixXi& GetCollisionFaceTopo() const {
		return _concept->GetCollisionFaceTopo();
	}


private:
	class Concept {
	public:
		virtual const BlockVector& GetCollisionVertexDerivative(int idx) const = 0;
		virtual Vector3d GetCollisionVertexVelocity(int idx) const = 0;
		virtual const MatrixXd& GetCollisionVertices() const = 0;
		virtual const MatrixXi& GetCollisionEdgeTopo() const = 0;
		virtual const MatrixXi& GetCollisionFaceTopo() const = 0;
	};

	template<class T>
	class Model : public Concept {
	public:
		Model(T* obj) : _obj(obj) {}
		const BlockVector & GetCollisionVertexDerivative(int idx) const override {
			return _obj->GetCollisionVertexDerivative(idx);
		}
		Vector3d GetCollisionVertexVelocity(int idx) const override {
			return _obj->GetCollisionVertexVelocity(idx);
		}
		const MatrixXd & GetCollisionVertices() const override {
			return _obj->GetCollisionVertices();
		}
		const MatrixXi & GetCollisionEdgeTopo() const override {
			return _obj->GetCollisionEdgeTopo();
		}
		const MatrixXi & GetCollisionFaceTopo() const override {
			return _obj->GetCollisionFaceTopo();
		}

	private:
		T* _obj;
	};

	Concept* _concept;
};

class IPCHelper {
public:
	IPCHelper(const json& config);
	virtual ~IPCHelper() = default;
	
	void SetObjects(
		const typename std::vector<CollisionInterface*>::const_iterator& begin,
		const typename std::vector<CollisionInterface*>::const_iterator& end
	);

	typedef std::function<bool(const CollisionInfo&)> Judger;

	double GetBarrierEnergy(const std::vector<CollisionInterface*>& interface) const;
	double GetBarrierEnergy() const;
	VectorXd GetBarrierEnergyGradient(const std::vector<CollisionInterface*>& interface) const;
	VectorXd GetBarrierEnergyGradient() const;
	void GetBarrierEnergyHessian(const std::vector<CollisionInterface*>& interface, COO &coo, int offset_x, int offset_y) const;
	void GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const;

    virtual void ComputeConstraintSet(const std::vector<CollisionInterface*>& interface, const VectorXd &x) = 0;
    void ComputeConstraintSet(const VectorXd &x);
    virtual double GetMaxStep(const std::vector<CollisionInterface*>& interface, const VectorXd& p) = 0;
    double GetMaxStep(const VectorXd& p);
	
    std::vector<CollisionInfo> _constraint_set;

protected:
    double GetVFBarrierEnergy(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    double GetEEBarrierEnergy(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Vector12d GetVFBarrierEnergyGradient(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Vector12d GetEEBarrierEnergyGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Matrix12d GetVFBarrierEnergyHessian(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Matrix12d GetEEBarrierEnergyHessian(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;

	std::vector<CollisionInterface*> _objs;

    double _d_hat;
	double _kappa;
};

template<class ConstraintSetGenerator, class MaxStepEstimator>
class ConcreteIPCHelper : public IPCHelper, public ConstraintSetGenerator, public MaxStepEstimator {
	ConcreteIPCHelper(const json& config) : IPCHelper(config), ConstraintSetGenerator(config["constraint-set-generator"]), MaxStepEstimator(config["max-step-generator"]) {}
	void ComputeConstraintSet(const VectorXd &x) override {
		ConstraintSetGenerator::ComputeConstraintSet(x, _objs, _d_hat, _constraint_set);
	}

	double GetMaxStep(const VectorXd &p) override {
		return MaxStepEstimator::GetMaxStep(p, _objs, _d_hat, _constraint_set);
	}
};