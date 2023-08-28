#include "Voxelizer.hpp"
#include <iostream>

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

using GridFunc = std::function<void(Vector3d& trilinear_coef, const Ref<const Eigen::RowVectorXi> indices)>;

void SimpleMeshVoxelizer::Voxelize(const Ref<const VectorXd> &x, const Ref<const MatrixXi> &face_topo, double grid_size, VectorXd &grid_vertices, MatrixXi &grid_topo, MatrixXi &grid_edge_topo, MatrixXi &grid_face_topo, VectorXi &vertices_grid_id, MatrixXd &tri_coefs) {
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
		1, _discrete_size[0], _discrete_size[0] * _discrete_size[1]
	};

	// face_local_id[dir][i]: the local id of the ith vertex of face[dir],
	//						  where face[dir] is the previous adjacent edge
	//						  in direction dir
	static const int face_local_id[6][4] = {
		{0, 4, 6, 2},
		{0, 1, 5, 4},
		{0, 2, 3, 1},
		{1, 3, 7, 5},
		{2, 6, 7, 3},
		{4, 5, 7, 6}
	};
	static const int dir_offset[3] = {1, 2, 4};
	
	visited_cnt = 0;
	int discrete_coord[3];
	int cur_grid_id = 0;
	for (discrete_coord[2] = 0; discrete_coord[2] < _discrete_size[2]; discrete_coord[2]++) {
		for (discrete_coord[1] = 0; discrete_coord[1] < _discrete_size[1]; discrete_coord[1]++) {
			for (discrete_coord[0] = 0; discrete_coord[0] < _discrete_size[0]; discrete_coord[0]++, cur_grid_id++) {
				if (visited[cur_grid_id]) {
					bool vertex_existed[8] = {};
					topo_id2grid_id[visited_cnt] = cur_grid_id;
					for (int dir = 0; dir < 3; dir++) {
						if (discrete_coord[dir] != 0 && visited[cur_grid_id - offset[dir]]) {
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
								(discrete_coord[0] + min_grid_coords[0] + (id & 1)) * grid_size,
								(discrete_coord[1] + min_grid_coords[1] + ((id >> 1) & 1)) * grid_size,
								(discrete_coord[2] + min_grid_coords[2] + ((id >> 2) & 1)) * grid_size
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

	// std::cerr << "Grid vertices: " << std::endl << StackVector<double, 3>(grid_vertices) << std::endl;
	// std::cerr << "Grid topo: " << std::endl << grid_topo << std::endl;

	visited_cnt = 0;
	cur_grid_id = 0;
	std::vector<RowVector2i> edge_list;
	std::vector<RowVector3i> face_list;
	for (discrete_coord[2] = 0; discrete_coord[2] < _discrete_size[2]; discrete_coord[2]++) {
		for (discrete_coord[1] = 0; discrete_coord[1] < _discrete_size[1]; discrete_coord[1]++) {
			for (discrete_coord[0] = 0; discrete_coord[0] < _discrete_size[0]; discrete_coord[0]++, cur_grid_id++) {
				if (visited[cur_grid_id]) {
					const auto& indices = grid_topo.row(visited_cnt);
					for (int dir = 0; dir < 3; dir++) {
						const auto rear_face_local_id = face_local_id[dir];
						if (discrete_coord[dir] == 0 || !visited[cur_grid_id - offset[dir]]) {
							// std::cerr << "Grid " << cur_grid_id << " direction " << dir << std::endl;
							face_list.push_back((RowVector3i() << 
								indices[rear_face_local_id[0]],
								indices[rear_face_local_id[1]],
								indices[rear_face_local_id[2]]
							).finished());
							face_list.push_back((RowVector3i() << 
								indices[rear_face_local_id[0]],
								indices[rear_face_local_id[2]],
								indices[rear_face_local_id[3]]
							).finished());
							for (int i = 0; i < 4; i++) {
								edge_list.push_back((RowVector2i() <<
									indices[rear_face_local_id[i]],
									indices[rear_face_local_id[(1 + i) & 3]]
								).finished());
							}
						}
						const auto front_face_local_id = face_local_id[dir + 3];
						if (discrete_coord[dir] == _discrete_size[dir] - 1 || !visited[cur_grid_id + offset[dir]]) {
							// std::cerr << "Grid " << cur_grid_id << " direction " << dir + 3 << std::endl;
							face_list.push_back((RowVector3i() << 
								indices[front_face_local_id[0]],
								indices[front_face_local_id[1]],
								indices[front_face_local_id[2]]
							).finished());
							face_list.push_back((RowVector3i() << 
								indices[front_face_local_id[0]],
								indices[front_face_local_id[2]],
								indices[front_face_local_id[3]]
							).finished());
							for (int i = 0; i < 4; i++) {
								edge_list.push_back((RowVector2i() <<
									indices[front_face_local_id[i]],
									indices[front_face_local_id[(1 + i) & 3]]
								).finished());
							}
						}
					}
					visited_cnt++;
				}
			}
		}
	}
	const int num_grid_edges = edge_list.size();
	const int num_grid_faces = face_list.size();
	grid_edge_topo.resize(edge_list.size(), 2);
	grid_face_topo.resize(face_list.size(), 3);
	for (int i = 0; i < num_grid_edges; i++) {
		grid_edge_topo.row(i) = edge_list[i];
	}
	for (int i = 0; i < num_grid_faces; i++) {
		grid_face_topo.row(i) = face_list[i];
	}

	vertices_grid_id.resize(num_points);
	tri_coefs.resize(num_points, 3);

	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		const Vector3d& position = x.segment<3>(i3);
		const auto position_discrete = GetVertexGrid(position);
		tri_coefs.row(i) = (position - position_discrete.cast<double>() * grid_size).array() / grid_size;
		vertices_grid_id(i) = std::find(
			topo_id2grid_id.begin(),
			topo_id2grid_id.end(),
			GetGridId(position_discrete - min_grid_coords)
		) - topo_id2grid_id.begin();
	}
}