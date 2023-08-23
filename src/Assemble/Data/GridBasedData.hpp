#pragma once

#include "Data.hpp"

template<class SampledProxyData>
struct GridBasedData : public BasicData {
	void Refresh();
	void Voxelization(VectorXd& grid_vertices, MatrixXi& grid_topo);

	SampledProxyData* _proxy;
	SparseMatrixXd _base_compressed;
	VectorXd _mass;
	int _num_grids;

private:
	//<- id of the grid
	int FindEmbeddingGrid(const VectorXd& grid_vertices, const MatrixXi& grid_topo, const Vector3d& x) {
		// TODO:
	}
};

template<class SampledProxyData>
void GridBasedData<SampledProxyData>::Refresh() {
	// TODO: Assume the grids are ordered alphabetically
	double grid_size;	// TODO: TBD
	VectorXd grid_vertices;
	MatrixXi grid_topo;
	Voxelization(grid_vertices, grid_topo);

	const VectorXd& x = _proxy->_x;

	/** TODO: change it to adapt
	  * Assuming the ordering of the grid vertices:
	  *      4 --- 6
	  *     /     / |
	  *    5 --- 7  |
	  *    |     |  2
	  *    |     | /
	  *    1 --- 3 
	  * the unseen number is 0
	  */

	const int num_grids = grid_vertices.size() / 3;
	const int num_points = _proxy->_num_points;
	_base_compressed.resize(num_points, num_grids);
	_base_compressed.setZero();
	COO coo;
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		Vector3d vertex = x.segment<3>(i3);
		int grid_id = FindEmbeddingGrid(grid_vertices, grid_topo, x.segment<3>(i3));
		const auto indices = grid_topo.row(grid_id);
		Vector3d min_coord = grid_vertices.segment<3>(indices[0] * 3);
		Vector3d trilinear_coef = (vertex - min_coord) / grid_size;
		for (int j = 0; j < 8; j++) {
			coo.push_back(Tripletd(i, indices[j], (
				  (j & 1 ? trilinear_coef[0] : 1 - trilinear_coef[0])
				* (j & 2 ? trilinear_coef[1] : 1 - trilinear_coef[1])
				* (j & 4 ? trilinear_coef[2] : 1 - trilinear_coef[2])
			)));
		}
	}
	_base_compressed.setFromTriplets(coo.begin(), coo.end());
}