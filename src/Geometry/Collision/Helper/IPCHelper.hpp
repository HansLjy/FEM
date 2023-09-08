#pragma once

#include "EigenAll.h"
#include "Collision/CollisionShape/CollisionShape.h"
#include "Collision/SpatialHashing/SpatialHashing.hpp"
#include "Collision/CCD/CCD.h"

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

class IPCHelper {
public:
	IPCHelper(const json& config);
	~IPCHelper();
	template<class Object> void SetObjects(const typename std::vector<Object*>::const_iterator& begin, const typename std::vector<Object*>::const_iterator& end);

    void ComputeConstraintSet(const VectorXd &x);
	double GetBarrierEnergy() const;
	VectorXd GetBarrierEnergyGradient() const;
	void GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const;

    double GetMaxStep(const VectorXd& p);
	
    std::vector<CollisionInfo> _constraint_set;

protected:
	double GetFullCCD(const VectorXd& p);
    double GetVFBarrierEnergy(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    double GetEEBarrierEnergy(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Vector12d GetVFBarrierEnergyGradient(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Vector12d GetEEBarrierEnergyGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Matrix12d GetVFBarrierEnergyHessian(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Matrix12d GetEEBarrierEnergyHessian(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;

	int _dof;
	std::vector<int> _dofs;
	std::vector<int> _offsets;
	std::vector<CollisionShapeInterface*> _objs;

    double _d_hat;
	double _kappa;
    CCD* _ccd;
	SpatialHashing<EdgePrimitiveInfo> _edge_hash_table;
	SpatialHashing<VertexPrimitiveInfo> _vertex_hash_table;

	unsigned int _time_stamp = 0;
};

template<class Object>
void IPCHelper::SetObjects(const typename std::vector<Object*>::const_iterator& begin, const typename std::vector<Object*>::const_iterator& end) {
	_objs.clear();
	_dofs.clear();
	_dof = 0;
	for (auto itr = begin; itr != end; ++itr) {
		_objs.push_back(*itr);
		_dofs.push_back((*itr)->GetDOF());
		_offsets.push_back(_dof);
		_dof += (*itr)->GetDOF();
	}
}
