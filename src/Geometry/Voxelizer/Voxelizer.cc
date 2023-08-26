#include "Voxelizer.hpp"
#include <iostream>

const Matrix<int, 12, 2> local_edge_topo = (Matrix<int, 12, 2>() <<
	0, 4,
	0, 1,
	1, 5,
	5, 4,
	4, 6,
	5, 7,
	1, 3,
	0, 2,
	2, 3,
	3, 7,
	7, 6,
	6, 2
).finished();

const Matrix<int, 12, 3> local_face_topo = (Matrix<int, 12, 3>() <<
	0, 5, 4,
	0, 1, 5,
	2, 6, 7,
	2, 7, 3,
	7, 6, 4,
	7, 4, 5,
	3, 1, 0,
	3, 0, 2,
	7, 5, 1,
	7, 1, 3,
	6, 2, 0,
	6, 0, 4
).finished();

MatrixXi Voxelizer::GetEdgeTopo(const MatrixXi& grid_topo) {
	int num_grids = grid_topo.rows();
	MatrixXi edge_topo(num_grids * 12, 2);
	for (int i = 0, i12 = 0; i < num_grids; i++, i12 += 12) {
		const auto& indices = grid_topo.row(i);
		for (int j = 0; j < 12; j++) {
			edge_topo.row(i12 + j) << indices[local_edge_topo(j, 0)], indices[local_edge_topo(j, 1)];
		}
	}
	return edge_topo;
}

MatrixXi Voxelizer::GetFaceTopo(const MatrixXi& grid_topo) {
	int num_grids = grid_topo.rows();
	MatrixXi face_topo(num_grids * 12, 3);
	for (int i = 0, i12 = 0; i < num_grids; i++, i12 += 12) {
		const auto& indices = grid_topo.row(i);
		for (int j = 0; j < 12; j++) {
			face_topo.row(i12 + j) << indices[local_face_topo(j, 0)], indices[local_face_topo(j, 1)], indices[local_face_topo(j, 2)];
		}
	}
	return face_topo;
}

Vector3i Voxelizer::GetVertexGrid(const Vector3d &coord) {
	Vector3i index;
	for (int i = 0; i < 3; i++) {
		int grid_id = std::floor(coord(i) / _grid_size);
		if (grid_id * _grid_size > coord[i]) {
			grid_id--;
		}
		index(i) = grid_id;
	}
	return index;
}

int Voxelizer::GetVertexGridId(const Vector3d &coord) {
	return GetGridId(GetVertexGrid(coord));
}

// x, y, z are shifted coordinate!
int Voxelizer::GetGridId(int x, int y, int z) {
	return x + (y + z * _discrete_size[1]) * _discrete_size[0];
}

// discrete_coord is unshifted coordinate
int Voxelizer::GetGridId(const Vector3i &discrete_coord) {
	return GetGridId(discrete_coord[0], discrete_coord[1], discrete_coord[2]);
}

void SimplePointCloudVoxelizer::Voxelize(const Ref<const VectorXd> &x, int num_points, double grid_size, VectorXd &grid_vertices, MatrixXi &grid_topo) {
	_grid_size = grid_size;
	assert(num_points > 0);
	Vector3d min_coord = x.segment<3>(0), max_coord = x.segment<3>(0);
	for (int i = 1, i3 = 3; i < num_points; i++, i3 += 3) {
		for (int j = 0; j < 3; j++) {
			if (x(i3 + j) < min_coord(j)) {
				min_coord(j) = x(i3 + j);
			}
			if (x(i3 + j) > max_coord(j)) {
				max_coord(j) = x(i3 + j);
			}
		}
	}
	Vector3i min_grid_coords = GetVertexGrid(min_coord);
	Vector3i max_grid_coords = GetVertexGrid(max_coord);

	for (int i = 0; i < 3; i++) {
		_discrete_size[i] = max_grid_coords(i) - min_grid_coords(i) + 1;
	}
	assert(1ll * _discrete_size[0] * _discrete_size[1] * _discrete_size[2] <= std::numeric_limits<unsigned int>::max());
	unsigned int total_grids = _discrete_size[0] * _discrete_size[1] * _discrete_size[2];

	std::vector<bool> visited(total_grids, false);

	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		visited[GetVertexGridId(x.segment<3>(i3))] = true;
	}

	// TODO:
}

void SimpleMeshVoxelizer::Voxelize(const Ref<const VectorXd> &x, const Ref<const MatrixXi> &face_topo, double grid_size, VectorXd &grid_vertices, MatrixXi &grid_topo) {
	_grid_size = grid_size;
	const int num_points = x.size() / 3;
	const int num_faces = face_topo.rows();

	assert(num_points > 0);
	Vector3i min_grid_coords = GetVertexGrid(x.segment<3>(0));
	Vector3i max_grid_coords = GetVertexGrid(x.segment<3>(0));
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		min_grid_coords = min_grid_coords.cwiseMin(GetVertexGrid(x.segment<3>(i3)));
		max_grid_coords = max_grid_coords.cwiseMax(GetVertexGrid(x.segment<3>(i3)));
	}

	for (int i = 0; i < 3; i++) {
		_discrete_size[i] = max_grid_coords[i] - min_grid_coords[i] + 1;
	}

	assert(1ll * _discrete_size[0] * _discrete_size[1] * _discrete_size[2] < std::numeric_limits<int>::max());
	int total_size = _discrete_size[0] * _discrete_size[1] * _discrete_size[2];
	std::vector<bool> visited(total_size, false);

	for (int i = 0; i < num_faces; i++) {
		const auto& indices = face_topo.row(i);
		const int id1 = indices[0], id2 = indices[1], id3 = indices[2];
		Vector3i x1_discrete = GetVertexGrid(x.segment<3>(id1 * 3));
		Vector3i x2_discrete = GetVertexGrid(x.segment<3>(id2 * 3));
		Vector3i x3_discrete = GetVertexGrid(x.segment<3>(id3 * 3));

		Vector3i min_coord_discrete = x1_discrete.cwiseMin(x2_discrete.cwiseMin(x3_discrete)) - min_grid_coords;
		Vector3i max_coord_discrete = x1_discrete.cwiseMax(x2_discrete.cwiseMax(x3_discrete)) - min_grid_coords;

		for (int x_discrete = min_coord_discrete[0]; x_discrete <= max_coord_discrete[0]; x_discrete++) {
			for (int y_discrete = min_coord_discrete[1]; y_discrete <= max_coord_discrete[1]; y_discrete++) {
				for (int z_discrete = min_coord_discrete[2]; z_discrete <= max_coord_discrete[2]; z_discrete++) {
					visited[(GetGridId(x_discrete, y_discrete, z_discrete))] = true;
				}
			}
		}
	}

	int visited_cnt = 0;
	for (int i = 0; i < total_size; i++) {
		if (visited[i]) {
			visited_cnt++;
		}
	}

	std::vector<int> topo_id2grid_id(visited_cnt);
	std::vector<Vector3d> grid_vertices_list;
	grid_topo.resize(visited_cnt, 8);

	int offset[] = {
		1, _discrete_size[0], _discrete_size[1] * _discrete_size[2]
	};

	// face_local_id[dir][i]: the local id of the ith vertex of face[dir],
	//						  where face[dir] is the previous adjacent edge
	//						  in direction dir
	static const int face_local_id[3][4] = {
		{0, 2, 4, 6},
		{0, 1, 4, 5},
		{0, 1, 2, 3}
	};
	static const int dir_offset[3] = {1, 2, 4};
	bool on_edge[3];
	visited_cnt = 0;
	for (int z_discrete = 0; z_discrete < _discrete_size[2]; z_discrete++) {
		on_edge[2] = (z_discrete == 0);
		for (int y_discrete = 0; y_discrete < _discrete_size[1]; y_discrete++) {
			on_edge[1] = (y_discrete == 0);
			for (int x_discrete = 0; x_discrete < _discrete_size[0]; x_discrete++) {
				on_edge[0] = (x_discrete == 0);
				int cur_grid_id = GetGridId(x_discrete, y_discrete, z_discrete);
				if (visited[cur_grid_id]) {
					bool vertex_existed[8] = {};
					topo_id2grid_id[visited_cnt] = cur_grid_id;
					for (int dir = 0; dir < 3; dir++) {
						if (!on_edge[dir] && visited[cur_grid_id - offset[dir]]) {
							auto cur_face_local_id = face_local_id[dir];
							int nbr_grid_id = std::find(
								topo_id2grid_id.begin(),
								topo_id2grid_id.end(),
								cur_grid_id - offset[dir]
							) - topo_id2grid_id.begin();
							for (int i = 0; i < 4; i++) {
								grid_topo(visited_cnt, cur_face_local_id[i]) = grid_topo(nbr_grid_id, cur_face_local_id[i] + dir_offset[dir]);
								vertex_existed[cur_face_local_id[i]] = true;
							}
						}
					}
					for (int id = 0; id < 8; id++) {
						if (!vertex_existed[id]) {
							grid_vertices_list.push_back((Vector3d() <<
								(x_discrete + (id & 1)) * grid_size,
								(y_discrete + ((id >> 1) & 1)) * grid_size,
								(z_discrete + ((id >> 2) & 1)) * grid_size
							).finished());
							grid_topo(visited_cnt, id) = grid_vertices_list.size() - 1;
						}
					}
					visited_cnt++;
				}
			}
		}
	}
	
	const int total_grid_vertices = grid_vertices_list.size();
	grid_vertices.resize(total_grid_vertices * 3);
	for (int i = 0, i3 = 0; i < total_grid_vertices; i++, i3 += 3) {
		grid_vertices.segment<3>(i3) = grid_vertices_list[i];
	}
}