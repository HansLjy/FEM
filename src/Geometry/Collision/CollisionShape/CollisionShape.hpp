//
// Created by hansljy on 12/21/22.
//
#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "BlockMatrix.h"

struct CollisionData {
	MatrixXd _collision_vertices;
	MatrixXd _collision_vertex_velocity;
	MatrixXi _collision_edge_topo;
	MatrixXi _collision_face_topo;
	std::vector<BlockVector> _vertex_derivatives;
};

class BasicCollisionShape {
public:
	template<class Data> Vector3d GetCollisionVertexVelocity(const Data* data, int idx) const {return data->_collision_vertex_velocity.row(idx);}
	template<class Data> const BlockVector& GetCollisionVertexDerivative(const Data* data, int idx) const {return data->_vertex_derivatives[idx];}
	template<class Data> static const MatrixXd& GetCollisionVertices(const Data* data) {return data->_collision_vertices;}
	template<class Data> static const MatrixXi& GetCollisionEdgeTopo(const Data* data) {return data->_collision_edge_topo;}
	template<class Data> static const MatrixXi& GetCollisionFaceTopo(const Data* data) {return data->_collision_face_topo;}
};

class SampledCollisionShape : public BasicCollisionShape {
public:
	SampledCollisionShape(const json& config) {}
	SampledCollisionShape() = default;
	template<class Data> static void Initialize(Data* obj);

	template<class Data> static void ComputeCollisionVertex(Data* obj, const Ref<const VectorXd>& x);
	template<class Data> static void ComputeCollisionVertexVelocity(Data* obj, const Ref<const VectorXd>& v);
	template<class Data> const MatrixXi& GetCollisionEdgeTopo(const Data* obj) const;
	template<class Data> const MatrixXi& GetCollisionFaceTopo(const Data* obj) const;
};

class NullCollisionShape : public BasicCollisionShape {
public:
	NullCollisionShape(const json& config) {}
	NullCollisionShape() = default;
	template<class Data> void Initialize(Data* data) {
		data->_collision_vertices = MatrixXd(0, 3);
		data->_collision_edge_topo = MatrixXi(0, 2);
		data->_collision_face_topo = MatrixXi(0, 3);
	}
	template<class Data> void ComputeCollisionVertex(Data* data, const Ref<const VectorXd>& x) {}
	template<class Data> void ComputeCollisionVertexVelocity(Data* data, const Ref<const VectorXd>& x) {}
	template<class Data> const BlockVector& GetCollisionVertexDerivative(const Data* data, int idx) const {return null_vector;}
	template<class Data> Vector3d GetCollisionVertexVelocity(const Data* data, int idx) const {return Vector3d::Zero();}

protected:
	const BlockVector null_vector{0, 0, {}, {}, MatrixXd(0, 3)};
};

#include "FixedShape/FixedShape.hpp"
class FixedCollisionShape : public BasicCollisionShape {
public:
	template<class Data>
	FixedCollisionShape(const Data* data, const json& config) : FixedCollisionShape(
		data,
		FixedShapeFactory::Instance()->GetVertices(config["type"], config),
		FixedShapeFactory::Instance()->GetEdgeTopo(config["type"], config),
		FixedShapeFactory::Instance()->GetFaceTopo(config["type"], config)
	) {}

	template<class Data>
	FixedCollisionShape(const Data* data, const MatrixXd& vertices, const MatrixXi& edge_topo, const MatrixXi& face_topo) {
		data->_collision_vertices = vertices;
		data->_collision_edge_topo = edge_topo;
		data->_collision_face_topo = face_topo;
		data->_collision_num_points = vertices.rows();
	}

	template<class Data> void Initialize(Data* obj) {}
	template<class Data> void ComputeCollisionVertex(Data* obj, const Ref<const VectorXd>& x) {}
	template<class Data> void ComputeCollisionVertexVelocity(Data* obj, const Ref<const VectorXd>& x) {}
	template<class Data> const BlockVector& GetCollisionVertexDerivative(const Data* obj, int idx) const {return null_vector;}
	template<class Data> double GetMaxVelocity(const Data* obj, const Ref<const VectorXd> &v) const {return 0;}
	template<class Data> Vector3d GetCollisionVertexVelocity(const Data* obj, const Ref<const VectorXd>& v, int idx) const {return Vector3d::Zero();}

protected:
	const BlockVector null_vector{0, 0, {}, {}, MatrixXd(0, 3)};
};

template<class Data>
void SampledCollisionShape::Initialize(Data* data) {
	for (int i = 0, i3 = 0; i < data->_num_points; i++, i3 += 3) {
		data->_vertex_derivatives.push_back(BlockVector(data->_dof, 1, {i3}, {3}, Matrix3d::Identity()));
	}
}

template<class Data>
void SampledCollisionShape::ComputeCollisionVertex(Data *obj, const Ref<const VectorXd> &x) {
	obj->_collision_vertices = StackVector<double, 3>(x);
}

template<class Data>
void SampledCollisionShape::ComputeCollisionVertexVelocity(Data *obj, const Ref<const VectorXd> &v) {
	obj->_collision_vertex_velocity = StackVector<double, 3>(v);
}

template<class Data>
const MatrixXi& SampledCollisionShape::GetCollisionEdgeTopo(const Data* data) const {
	return data->_edge_topo;
}

template<class Data>
const MatrixXi& SampledCollisionShape::GetCollisionFaceTopo(const Data* data) const {
	return data->_face_topo;
}
