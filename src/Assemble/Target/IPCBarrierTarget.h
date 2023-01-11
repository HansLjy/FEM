//
// Created by hansljy on 12/2/22.
//

#ifndef FEM_IPCBARRIERTARGET_H
#define FEM_IPCBARRIERTARGET_H

#include "Collision/SpatialHashing/SpatialHashing.hpp"
#include "Collision/CCD/CCD.h"
#include "EigenAll.h"
#include "Object.h"
#include "Target.h"

#include <vector>

class CollisionCulling;

enum class CollisionType {
	kVertexFace,
	kEdgeEdge
};

struct CollisionInfo {
	CollisionType _type;
	int _obj_id1;
	int _obj_id2;
	int _primitive_id1;
	int _primitive_id2;
};

struct EdgePrimitiveInfo {
    int _obj_id;
    int _primitive_id;
	Vector3d _vertex1;
	Vector3d _vertex2;

	bool operator<(const EdgePrimitiveInfo& rhs) const {
		return _obj_id < rhs._obj_id || (_obj_id == rhs._obj_id && _primitive_id < rhs._primitive_id);
	}

	bool operator==(const EdgePrimitiveInfo& rhs) const {
		return _obj_id == rhs._obj_id && _primitive_id == rhs._primitive_id;
	}
};

struct VertexPrimitiveInfo {
    int _obj_id;
    int _primitive_id;
	Vector3d _vertex;

	bool operator<(const VertexPrimitiveInfo& rhs) const {
		return _obj_id < rhs._obj_id || (_obj_id == rhs._obj_id && _primitive_id < rhs._primitive_id);
	}

	bool operator==(const VertexPrimitiveInfo& rhs) const {
		return _obj_id == rhs._obj_id && _primitive_id == rhs._primitive_id;
	}
};

class IPCBarrierTarget : public Target {
public:
    IPCBarrierTarget(const std::vector<Object*>& objs, int begin, int end, const json& config);

    /**
     * @note This function will actually compute the collision shape of
     *       every object in the system, so you don't need to call other
     *       function to do the job
     */
    void ComputeConstraintSet(const Eigen::VectorXd &x);

    double GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const override;
    void GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x, Ref<Eigen::VectorXd> gradient) const override;
    void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const override;

    double GetMaxStep(const VectorXd& p);

protected:
	double GetFullCCD(const VectorXd& p);

	double GetBarrierEnergy() const;
	VectorXd GetBarrierEnergyGradient() const;
	void GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const;

    double GetVFBarrierEnergy(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    double GetEEBarrierEnergy(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Vector12d GetVFBarrierEnergyGradient(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Vector12d GetEEBarrierEnergyGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Matrix12d GetVFBarrierEnergyHessian(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Matrix12d GetEEBarrierEnergyHessian(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;

    const double _d_hat;
	double _kappa;
    std::vector<CollisionInfo> _constraint_set;
    CCD* _ccd;
	SpatialHashing<EdgePrimitiveInfo> _edge_hash_table;
	SpatialHashing<VertexPrimitiveInfo> _vertex_hash_table;

	unsigned int _time_stamp = 0;
};

#endif //FEM_IPCBARRIERTARGET_H
