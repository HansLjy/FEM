#include "CoarseGridBasedData.hpp"
#include "Voxelizer/Voxelizer.hpp"

void CoarseGridBasedData::Refresh() {
	PointCloudVoxelizer voxelizer;
	voxelizer.Voxelize(_proxy._x, _proxy._num_points, _x, _edge_topo);
	_num_points = _x.size() / 3;
}