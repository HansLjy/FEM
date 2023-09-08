//
// Created by hansljy on 12/21/22.
//
#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "BlockMatrix.h"

class CollisionShapeInterface {
public:
	virtual double GetMaxVelocity(const Ref<const VectorXd>& v) const = 0;
    virtual void ComputeCollisionShape(const Ref<const VectorXd>& x) = 0;
	virtual const BlockVector& GetVertexDerivative(int idx) const = 0;
	virtual Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd>& v, int idx) const = 0;
    virtual const MatrixXd& GetCollisionVertices() const = 0;
    virtual const MatrixXi& GetCollisionEdgeTopo() const = 0;
    virtual const MatrixXi& GetCollisionFaceTopo() const = 0;
	virtual int GetCollisionVerticeNumber() const = 0;
};

class BasicCollisionShape {
public:
	template<class Data> const MatrixXd& GetCollisionVertices(const Data* obj) const {return _collision_vertices;}
	template<class Data> const MatrixXi& GetCollisionEdgeTopo(const Data* obj) const {return _collision_edge_topo;}
	template<class Data> const MatrixXi& GetCollisionFaceTopo(const Data* obj) const {return _collision_face_topo;}
	template<class Data> int GetCollisionVerticeNumber(const Data* obj) const {return _collision_num_points;}

protected:
	int _collision_num_points;
	MatrixXd _collision_vertices;
	MatrixXi _collision_edge_topo;
	MatrixXi _collision_face_topo;
};

class SampledCollisionShape : public BasicCollisionShape {
public:
	SampledCollisionShape(const json& config) {}
	SampledCollisionShape() = default;
	template<class Data> void Initialize(const Data* obj);
	template<class Data> void ComputeCollisionShape(const Data* obj, const Ref<const VectorXd>& x);
	template<class Data> const BlockVector& GetVertexDerivative(const Data* obj, int idx) const;
	template<class Data> double GetMaxVelocity(const Data* obj, const Ref<const VectorXd> &v) const;
	template<class Data> Vector3d GetCollisionVertexVelocity(const Data* obj, const Ref<const VectorXd>& v, int idx) const;
	template<class Data> int GetCollisionVerticeNumber(const Data* obj) const;
	template<class Data> const MatrixXi& GetCollisionEdgeTopo(const Data* obj) const;
	template<class Data> const MatrixXi& GetCollisionFaceTopo(const Data* obj) const;

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

class NullCollisionShape : public BasicCollisionShape {
public:
	NullCollisionShape(const json& config) {}
	NullCollisionShape() = default;
	template<class Data> void Initialize(const Data* data) {
		_collision_num_points = 0;
		_collision_vertices = MatrixXd(0, 3);
		_collision_edge_topo = MatrixXi(0, 2);
		_collision_face_topo = MatrixXi(0, 3);
	}
	template<class Data> void ComputeCollisionShape(const Data* data, const Ref<const VectorXd>& x) {}
	template<class Data> const BlockVector& GetVertexDerivative(const Data* data, int idx) const {return null_vector;}
	template<class Data> double GetMaxVelocity(const Data* data, const Ref<const VectorXd> &v) const {return 0;}
	template<class Data> Vector3d GetCollisionVertexVelocity(const Data* data, const Ref<const VectorXd>& v, int idx) const {return Vector3d::Zero();}

protected:
	const BlockVector null_vector{0, 0, {}, {}, MatrixXd(0, 3)};
};

#include "FixedShape/FixedShape.hpp"
class FixedCollisionShape : public BasicCollisionShape {
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
	template<class FixedObject> void Initialize(const FixedObject* obj) {}
	template<class FixedObject> void ComputeCollisionShape(const FixedObject* obj, const Ref<const VectorXd>& x) {}
	template<class FixedObject> const BlockVector& GetVertexDerivative(const FixedObject* obj, int idx) const {return null_vector;}
	template<class FixedObject> double GetMaxVelocity(const FixedObject* obj, const Ref<const VectorXd> &v) const {return 0;}
	template<class FixedObject> Vector3d GetCollisionVertexVelocity(const FixedObject* obj, const Ref<const VectorXd>& v, int idx) const {return Vector3d::Zero();}

protected:
	const BlockVector null_vector{0, 0, {}, {}, MatrixXd(0, 3)};
};

template<class Data>
void SampledCollisionShape::Initialize(const Data* data) {
	for (int i = 0, i3 = 0; i < data->_num_points; i++, i3 += 3) {
		_vertex_derivatives.push_back(BlockVector(data->_dof, 1, {i3}, {3}, Matrix3d::Identity()));
	}
}

template<class Data>
void SampledCollisionShape::ComputeCollisionShape(const Data* data, const Ref<const VectorXd>& x) {
	_collision_vertices = StackVector<double, 3>(x);
}

template<class Data>
const BlockVector& SampledCollisionShape::GetVertexDerivative(const Data* data, int idx) const {
	return _vertex_derivatives[idx];
}

template<class Data>
double SampledCollisionShape::GetMaxVelocity(const Data* data, const Ref<const VectorXd> &v) const {
	double max_velocity = 0;
	for(int i = 0, j = 0; i < data->_num_points; i++, j += 3) {
		max_velocity = std::max(max_velocity, v.segment(j, 3).norm());
	}
	return max_velocity;
}

template<class Data>
Vector3d SampledCollisionShape::GetCollisionVertexVelocity(const Data* data, const Ref<const VectorXd>& v, int idx) const {
	return v.segment<3>(3 * idx);
}

template<class Data>
const MatrixXi& SampledCollisionShape::GetCollisionEdgeTopo(const Data* data) const {
	return data->_edge_topo;
}

template<class Data>
const MatrixXi& SampledCollisionShape::GetCollisionFaceTopo(const Data* data) const {
	return data->_face_topo;
}

template<class Data>
int SampledCollisionShape::GetCollisionVerticeNumber(const Data* data) const {
	return data->_num_points;
}
