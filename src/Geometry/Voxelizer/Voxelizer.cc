#include "Voxelizer.hpp"

Vector3i SimplePointCloudVoxelizer::GetVertexGrid(const Vector3d &coord) {
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

int SimplePointCloudVoxelizer::GetVertexGridId(const Vector3d &coord) {
	return GetGridId(GetVertexGrid(coord));
}

int SimplePointCloudVoxelizer::GetGridId(const Vector3i &discrete_coord) {
	Vector3i shifted_coord = discrete_coord - _min_grid_coords;
	return shifted_coord(0) + (shifted_coord(1) + shifted_coord(2) * _discrete_size[1]) * _discrete_size[0];
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
	_min_grid_coords = GetVertexGrid(min_coord);
	_max_grid_coords = GetVertexGrid(max_coord);

	for (int i = 0; i < 3; i++) {
		_discrete_size[i] = _max_grid_coords(i) - _min_grid_coords(i) + 1;
	}
	assert(1ll * _discrete_size[0] * _discrete_size[1] * _discrete_size[2] <= std::numeric_limits<unsigned int>::max());
	unsigned int total_grids = _discrete_size[0] * _discrete_size[1] * _discrete_size[2];

	std::vector<bool> visited(total_grids, false);

	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		visited[GetVertexGridId(x.segment<3>(i3))] = true;
	}

	// TODO:
}