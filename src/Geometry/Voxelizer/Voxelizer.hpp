#pragma once
#include "EigenAll.h"

// Voxelizer for point cloud
class PointCloudVoxelizer {
public:
	virtual void Voxelize(const Ref<const VectorXd>& x, int num_points, double grid_size, VectorXd& grid_vertices, MatrixXi& grid_topo);	
};

class SimplePointCloudVoxelizer : public PointCloudVoxelizer {
public:
	void Voxelize(const Ref<const VectorXd> &x, int num_points, double grid_size, VectorXd &grid_vertices, MatrixXi &grid_topo) override;

protected:
	double _grid_size;
	Vector3i _min_grid_coords, _max_grid_coords;
	unsigned int _discrete_size[3]; // number of maximum grids in all direction

	Vector3i GetVertexGrid(const Vector3d& coord);
	int GetVertexGridId(const Vector3d& coord);
	int GetGridId(const Vector3i& discrete_coord);
};

class MeshVoxelizer {
public:
	virtual void Voxelize(const Ref<const VectorXd>& x, const Ref<const MatrixXi>& topo, int num_points, double grid_size, VectorXd& grid_vertices, MatrixXi& grid_topo);
};

// A very coarse voxelizer
class SimpleMeshVoxelizer : public MeshVoxelizer {
public:
	void Voxelize(const Ref<const VectorXd> &x, const Ref<const MatrixXi> &topo, int num_points, double grid_size, VectorXd &grid_vertices, MatrixXi &grid_topo) override;
};