//
// Created by hansljy on 12/21/22.
//
#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "BlockMatrix.h"

class CollisionShape {
public:
	template<class Object> void Precompute(const Object* obj);
	template<class Object> void ComputeCollisionShape(const Object* obj, const Ref<const VectorXd>& x);
	template<class Object> const BlockVector& GetVertexDerivative(const Object* obj, int idx) const;
	template<class Object> double GetMaxVelocity(const Object* obj, const Ref<const VectorXd> &v) const;
	template<class Object> Vector3d GetCollisionVertexVelocity(const Object* obj, const Ref<const VectorXd>& v, int idx) const;
	template<class Object> const MatrixXd& GetCollisionVertices(const Object* obj) const {return _collision_vertices;}
	template<class Object> const MatrixXi& GetCollisionEdgeTopo(const Object* obj) const {return _collision_edge_topo;}
	template<class Object> const MatrixXi& GetCollisionFaceTopo(const Object* obj) const {return _collision_face_topo;}
	template<class Object> int GetCollisionVerticeNumber(const Object* obj) const {return _collision_num_points;}

protected:
	int _collision_num_points;
	MatrixXd _collision_vertices;
	MatrixXi _collision_edge_topo;
	MatrixXi _collision_face_topo;
};

#define PROXY_COLLISION_SHAPE(CollisionShape) \
	double GetMaxVelocity(const Ref<const VectorXd>& v) const override {\
		return CollisionShape::GetMaxVelocity(this, v);\
	}\
	void ComputeCollisionShape(const Ref<const VectorXd>& x) override {\
		CollisionShape::ComputeCollisionShape(this, x);\
	}\
	const BlockVector& GetVertexDerivative(int idx) const override {\
		return CollisionShape::GetVertexDerivative(this, idx);\
	}\
	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd>& v, int idx) const override{\
		return CollisionShape::GetCollisionVertexVelocity(this, v, idx);\
	}\
	const MatrixXd& GetCollisionVertices() const override {\
		return CollisionShape::GetCollisionVertices(this);\
	}\
	const MatrixXi& GetCollisionEdgeTopo() const override {\
		return CollisionShape::GetCollisionEdgeTopo(this);\
	}\
	const MatrixXi& GetCollisionFaceTopo() const override {\
		return CollisionShape::GetCollisionFaceTopo(this);\
	}\
	int GetCollisionVerticeNumber() const override {\
		return CollisionShape::GetCollisionVerticeNumber(this);\
	}\
	friend CollisionShape;

class SampledCollisionShape : public CollisionShape {
public:
	template<class Object> void Precompute(const Object* obj);
	template<class Object> void ComputeCollisionShape(const Object* obj, const Ref<const VectorXd>& x);
	template<class Object> const BlockVector& GetVertexDerivative(const Object* obj, int idx) const;
	template<class Object> double GetMaxVelocity(const Object* obj, const Ref<const VectorXd> &v) const;
	template<class Object> Vector3d GetCollisionVertexVelocity(const Object* obj, const Ref<const VectorXd>& v, int idx) const;
	template<class Object> int GetCollisionVerticeNumber(const Object* obj) const;
	template<class Object> const MatrixXi& GetCollisionEdgeTopo(const Object* obj) const;
	template<class Object> const MatrixXi& GetCollisionFaceTopo(const Object* obj) const;

protected:
	std::vector<BlockVector> _vertex_derivatives;
};

// class AffineDecomposedObject;

// class AffineDecomposedCollisionShape : public CollisionShape {
// public:
// 	void ComputeCollisionShape(const Ref<const VectorXd> &x) override;
// 	const MatrixXd & GetCollisionVertices() const override;
// 	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const override;
// 	const BlockVector & GetVertexDerivative(int idx) const override;

// 	void UpdateDerivative();

// protected:
// 	AffineDecomposedObject* _obj;
// 	std::vector<BlockVector> _children_vertex_derivative;
// 	std::vector<int> _children_vertices_offset;	// sum of #vertices for first N child
// };

class NullCollisionShape : public CollisionShape {
public:
	template<class NullObject> void Precompute(const NullObject* obj) {
		_collision_num_points = 0;
		_collision_vertices = MatrixXd(0, 3);
		_collision_edge_topo = MatrixXi(0, 2);
		_collision_face_topo = MatrixXi(0, 3);
	}
	template<class NullObject> void ComputeCollisionShape(const NullObject* obj, const Ref<const VectorXd>& x) {}
	template<class NullObject> const BlockVector& GetVertexDerivative(const NullObject* obj, int idx) const {return null_vector;}
	template<class NullObject> double GetMaxVelocity(const NullObject* obj, const Ref<const VectorXd> &v) const {return 0;}
	template<class NullObject> Vector3d GetCollisionVertexVelocity(const NullObject* obj, const Ref<const VectorXd>& v, int idx) const {return Vector3d::Zero();}

protected:
	const BlockVector null_vector{0, 0, {}, {}, MatrixXd(0, 3)};
};

#include "FixedShape/FixedShape.hpp"
class FixedCollisionShape : public CollisionShape {
public:
	FixedCollisionShape(const json& config) : FixedCollisionShape(
		FixedShapeFactory::Instance()->GetVertices(config["type"], config),
		FixedShapeFactory::Instance()->GetEdgeTopo(config["type"], config),
		FixedShapeFactory::Instance()->GetFaceTopo(config["type"], config)
	) {}

	FixedCollisionShape(const MatrixXd& vertices, const MatrixXi& edge_topo, const MatrixXi& face_topo) {
		_collision_vertices = vertices;
		_collision_edge_topo = edge_topo;
		_collision_face_topo = face_topo;
		_collision_num_points = vertices.rows();
	}
	template<class FixedObject> void Precompute(const FixedObject* obj) {}
	template<class FixedObject> void ComputeCollisionShape(const FixedObject* obj, const Ref<const VectorXd>& x) {}
	template<class FixedObject> const BlockVector& GetVertexDerivative(const FixedObject* obj, int idx) const {return null_vector;}
	template<class FixedObject> double GetMaxVelocity(const FixedObject* obj, const Ref<const VectorXd> &v) const {return 0;}
	template<class FixedObject> Vector3d GetCollisionVertexVelocity(const FixedObject* obj, const Ref<const VectorXd>& v, int idx) const {return Vector3d::Zero();}

protected:
	const BlockVector null_vector{0, 0, {}, {}, MatrixXd(0, 3)};
};

template<class SampledObject>
void SampledCollisionShape::Precompute(const SampledObject* obj) {
	for (int i = 0, i3 = 0; i < obj->_num_points; i++, i3 += 3) {
		_vertex_derivatives.push_back(BlockVector(obj->_dof, 1, {i3}, {3}, Matrix3d::Identity()));
	}
}

template<class SampledObject>
void SampledCollisionShape::ComputeCollisionShape(const SampledObject* obj, const Ref<const VectorXd>& x) {
	_collision_vertices = StackVector<double, 3>(x);
}

template<class SampledObject>
const BlockVector& SampledCollisionShape::GetVertexDerivative(const SampledObject* obj, int idx) const {
	return _vertex_derivatives[idx];
}

template<class SampledObject>
double SampledCollisionShape::GetMaxVelocity(const SampledObject* obj, const Ref<const VectorXd> &v) const {
	double max_velocity = 0;
	for(int i = 0, j = 0; i < obj->_num_points; i++, j += 3) {
		max_velocity = std::max(max_velocity, v.segment(j, 3).norm());
	}
	return max_velocity;
}

template<class SampledObject>
Vector3d SampledCollisionShape::GetCollisionVertexVelocity(const SampledObject* obj, const Ref<const VectorXd>& v, int idx) const {
	return v.segment<3>(3 * idx);
}

template<class SampledObject>
const MatrixXi& SampledCollisionShape::GetCollisionEdgeTopo(const SampledObject* obj) const {
	return obj->_edge_topo;
}

template<class SampledObject>
const MatrixXi& SampledCollisionShape::GetCollisionFaceTopo(const SampledObject* obj) const {
	return obj->_face_topo;
}

template<class SampledObject>
int SampledCollisionShape::GetCollisionVerticeNumber(const SampledObject* obj) const {
	return obj->_num_points;
}


