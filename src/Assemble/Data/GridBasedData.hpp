#pragma once

#include "Data.hpp"
#include "Collision/SpatialHashing/SpatialHashing.hpp"
#include <limits>

template<class SampledProxyData>
struct GridBasedData : public BasicData {
	void Refresh();

	SampledProxyData* _proxy;
	SparseMatrixXd _base_compressed;
	VectorXd _mass;
	int _num_grids;
	int _hash_table_size;


private:
	//<- id of the grid
	void Voxelization(VectorXd& grid_vertices, MatrixXi& grid_topo, double grid_size);
	static Vector3i GetGridIndex(const Vector3d& position, double grid_size, const Vector3i& offset) {
		Vector3i index;
		for (int i = 0; i < 3; i++) {
			index(i) = floor(position(i) / grid_size);
		}
		return index + offset;
	}
};

struct GridInfo {
	GridInfo(int vertex_id, const Eigen::Vector3<long long>& grid_index) : _vertex_id(vertex_id), _grid_index(grid_index) {}

	int _vertex_id;
	Vector3i _grid_index;	// (X, Y, Z) stands for the region of
							// [X, X + grid_size) x ... x ...
	
	bool operator<(const GridInfo& rhs) const {
		for (int i = 0; i < 3; i++) {
			if (_grid_index[i] != rhs._grid_index[i]) {
				return _grid_index[i] < rhs._grid_index[i];
			}
		}
		return false;
	}
};

template<class SampledProxyData>
void GridBasedData<SampledProxyData>::Voxelization(VectorXd& grid_vertices, MatrixXi& grid_topo, double grid_size) {
	const VectorXd& x = _proxy->_x;
	const int num_points = _proxy->_num_points;
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
	Vector3i min_coord_discrete = GetGridIndex(min_coord, grid_size, Vector3i::Zero());
	Vector3i max_coord_discrete = GetGridIndex(max_coord, grid_size, Vector3i::Zero());
	Vector3i offset = -min_coord_discrete;

	int dim[3] = {
		max_coord_discrete(0) - min_coord_discrete(0) + 1,
		max_coord_discrete(1) - min_coord_discrete(1) + 1,
		max_coord_discrete(2) - min_coord_discrete(2) + 1
	};
	assert(1ll * dim[0] * dim[1] * dim[2] <= std::numeric_limits<unsigned int>::max());
	unsigned int total_grids = dim[0] * dim[1] * dim[2];

	std::vector<bool> visited(total_grids);

	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		visited[i] = false;
	}
}

template<class SampledProxyData>
void GridBasedData<SampledProxyData>::Refresh() {
	// TODO: Assume the grids are ordered alphabetically
}