#pragma once

#include "EigenAll.h"
#include "BlockMatrix.h"
#include "JsonUtil.h"

class GridCollisionShape {
public:
	GridCollisionShape(const json& config) {}
	GridCollisionShape() = default;
	template<class Data> void Initialize(Data* data);
	template<class Data> double GetMaxVelocity(const Data* data, const Ref<const VectorXd> &v) const;
	template<class Data> void ComputeCollisionShape(const Data* data, const Ref<const VectorXd> &x);
	template<class Data> const BlockVector & GetCollisionVertexDerivative(const Data* data, int idx) const;
	template<class Data> Vector3d GetCollisionVertexVelocity(const Data* data, const Ref<const VectorXd> &v, int idx) const;
	template<class Data> const MatrixXd & GetCollisionVertices(const Data* data) const;
	template<class Data> const MatrixXi & GetCollisionEdgeTopo(const Data* data) const;
	template<class Data> const MatrixXi & GetCollisionFaceTopo(const Data* data) const;
	template<class Data> int GetCollisionVerticeNumber(const Data* data) const;
	template<class Data> void Recompute(const Data* data);

private:
	MatrixXd _collision_vertices;
	std::vector<BlockVector> _vertex_derivatives;
	MatrixXi _collision_edge_topo;
	MatrixXi _collision_face_topo;
};

template<class Data>
void GridCollisionShape::Initialize(Data* data) {}

template<class Data>
void GridCollisionShape::Recompute(const Data* data) {
	const int dof = data->_dof;
	const int num_points = data->_proxy->_num_points;
	if (_vertex_derivatives.size() != num_points) {
		_vertex_derivatives.resize(num_points);
	}
	const VectorXi& vertices_grid_id = data->_vertices_grid_id;
	const MatrixXd& trilinear_coef = data->_trilinear_coef;
	const MatrixXi& grid_topo = data->_grid_topo;
	for (int i = 0; i < num_points; i++) {
		int grid_id = vertices_grid_id(i);
		const Eigen::RowVector3d tri_coef = trilinear_coef.row(i);
		const RowVector8i indices = grid_topo.row(grid_id);
		
		Matrix<double, 24, 3> submatrix;
		std::vector<int> offsets(8);
		std::vector<int> segment_lengths(8);
		for (int id = 0; id < 8; id++) {
			submatrix.block<3, 3>(id * 3, 0) = 
				Matrix3d::Identity()
				* ((id & 1) ? tri_coef[0] : (1 - tri_coef[0]))
				* ((id & 2) ? tri_coef[1] : (1 - tri_coef[1]))
				* ((id & 4) ? tri_coef[2] : (1 - tri_coef[2]));
			offsets[id] = indices[id] * 3;
			segment_lengths[id] = 3;
		}
		_vertex_derivatives[i] = BlockVector(dof, 8, offsets, segment_lengths, submatrix);
	}

	_collision_edge_topo = data->_proxy->_edge_topo.topRows(data->_proxy->_num_edges);
	_collision_face_topo = data->_proxy->_face_topo.topRows(data->_proxy->_num_faces);
}

template<class Data>
double GridCollisionShape::GetMaxVelocity(const Data* data, const Ref<const VectorXd> &v) const {
	// This is only an approximation
	const int num_grid_points = data->_num_points;
	double max_velocity = 0;
	for (int i = 0, i3 = 0; i < num_grid_points; i++, i3 += 3) {
		max_velocity = std::max(v.segment<3>(i3).squaredNorm(), max_velocity);
	}
	return sqrt(max_velocity);
}

template<class Data>
void GridCollisionShape::ComputeCollisionShape(const Data* data, const Ref<const VectorXd> &x) {
	const int num_points = data->_proxy->_num_points;
	_collision_vertices.resize(num_points, 3);
	const VectorXi& vertices_grid_id = data->_vertices_grid_id;
	const MatrixXd& trilinear_coef = data->_trilinear_coef;
	const MatrixXi& grid_topo = data->_grid_topo;
	for (int i = 0; i < num_points; i++) {
		int grid_id = vertices_grid_id(i);
		const Eigen::RowVector3d tri_coef = trilinear_coef.row(i);
		const RowVector8i indices = grid_topo.row(grid_id);
		Vector3d vertex = Vector3d::Zero();
		for (int id = 0; id < 8; id++) {
			vertex += x.segment<3>(indices(id) * 3)
				* ((id & 1) ? tri_coef[0] : (1 - tri_coef[0]))
				* ((id & 2) ? tri_coef[1] : (1 - tri_coef[1]))
				* ((id & 4) ? tri_coef[2] : (1 - tri_coef[2]));
		}
		_collision_vertices.row(i) = vertex;
	}
}

template<class Data>
const BlockVector& GridCollisionShape::GetCollisionVertexDerivative(const Data* data, int idx) const {
	return _vertex_derivatives[idx];
}

template<class Data>
Vector3d GridCollisionShape::GetCollisionVertexVelocity(const Data* data, const Ref<const VectorXd> &v, int idx) const {
	int grid_id = data->_vertices_grid_id(idx);
	const Eigen::RowVector3d tri_coef = data->_trilinear_coef.row(idx);
	const RowVector8i indices = data->_grid_topo.row(grid_id);
	Vector3d velocity = Vector3d::Zero();
	for (int id = 0; id < 8; id++) {
		velocity += v.segment<3>(indices(id) * 3)
			* ((id & 1) ? tri_coef[0] : (1 - tri_coef[0]))
			* ((id & 2) ? tri_coef[1] : (1 - tri_coef[1]))
			* ((id & 4) ? tri_coef[2] : (1 - tri_coef[2]));
	}
	return velocity;
}

template<class Data>
const MatrixXd& GridCollisionShape::GetCollisionVertices(const Data* data) const {
	return _collision_vertices;
}

template<class Data>
const MatrixXi& GridCollisionShape::GetCollisionEdgeTopo(const Data* data) const {
	return _collision_edge_topo;
}

template<class Data>
const MatrixXi& GridCollisionShape::GetCollisionFaceTopo(const Data* data) const {
	return _collision_face_topo;
}

template<class Data>
int GridCollisionShape::GetCollisionVerticeNumber(const Data* data) const {
	return data->_proxy->_num_points;
}
