#pragma once

#include "EigenAll.h"
#include "BlockMatrix.h"
#include "JsonUtil.h"
#include "CollisionShape.hpp"

class GridCollisionShape : public BasicCollisionShape {
public:
	GridCollisionShape(const json& config) {}
	GridCollisionShape() = default;
	template<class Data> void Initialize(Data* data);
	template<class Data> void ComputeCollisionVertex(Data* data, const Ref<const VectorXd> &x);
	template<class Data> void ComputeCollisionVertexVelocity(Data* data, const Ref<const VectorXd> &v);
	template<class Data> void Recompute(const Data* data);
};

template<class Data>
void GridCollisionShape::Initialize(Data* data) {}

template<class Data>
void GridCollisionShape::Recompute(const Data* data) {
	const int dof = data->_dof;
	const int num_points = data->_proxy->_num_points;
	if (data->_vertex_derivatives.size() != num_points) {
		data->_vertex_derivatives.resize(num_points);
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
		data->_vertex_derivatives[i] = BlockVector(dof, 8, offsets, segment_lengths, submatrix);
	}

	data->_collision_edge_topo = data->_proxy->_edge_topo.topRows(data->_proxy->_num_edges);
	data->_collision_face_topo = data->_proxy->_face_topo.topRows(data->_proxy->_num_faces);
}

template<class Data>
void GridCollisionShape::ComputeCollisionVertex(Data* data, const Ref<const VectorXd> &x) {
	const int num_points = data->_proxy->_num_points;
	data->_collision_vertices.resize(num_points, 3);
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
		data->_collision_vertices.row(i) = vertex.transpose();
	}
}


template<class Data>
void GridCollisionShape::ComputeCollisionVertexVelocity(Data* data, const Ref<const VectorXd> &v) {
	const int num_points = data->_proxy->_num_points;
	data->_collision_vertex_velocity.resize(num_points, 3);
	const VectorXi& vertices_grid_id = data->_vertices_grid_id;
	const MatrixXd& trilinear_coef = data->_trilinear_coef;
	const MatrixXi& grid_topo = data->_grid_topo;
	for (int i = 0; i < num_points; i++) {
		int grid_id = vertices_grid_id(i);
		const Eigen::RowVector3d tri_coef = trilinear_coef.row(i);
		const RowVector8i indices = grid_topo.row(grid_id);
		Vector3d vertex_velocity = Vector3d::Zero();
		for (int id = 0; id < 8; id++) {
			vertex_velocity += v.segment<3>(indices(id) * 3)
				* ((id & 1) ? tri_coef[0] : (1 - tri_coef[0]))
				* ((id & 2) ? tri_coef[1] : (1 - tri_coef[1]))
				* ((id & 4) ? tri_coef[2] : (1 - tri_coef[2]));
		}
		data->_collision_vertex_velocity.row(i) = vertex_velocity.transpose();
	}
}