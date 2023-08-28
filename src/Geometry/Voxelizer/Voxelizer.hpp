#pragma once
#include "EigenAll.h"

class Voxelizer {
protected:
	double _grid_size;
	int _discrete_size[3]; // number of maximum grids in all direction

	Vector3i GetVertexGrid(const Vector3d& coord);
	int GetVertexGridId(const Vector3d& coord);
	int GetGridId(int x, int y, int z);
	int GetGridId(const Vector3i& discrete_coord);
};

// Voxelizer for point cloud
class PointCloudVoxelizer : public Voxelizer {
public:
	virtual void Voxelize(const Ref<const VectorXd>& x, int num_points, double grid_size, VectorXd& grid_vertices, MatrixXi& grid_topo) = 0;
};

class SimplePointCloudVoxelizer : public PointCloudVoxelizer {
public:
	void Voxelize(const Ref<const VectorXd> &x, int num_points, double grid_size, VectorXd &grid_vertices, MatrixXi &grid_topo) override;

protected:
};

class MeshVoxelizer : public Voxelizer {
public:
	virtual void Voxelize(const Ref<const VectorXd>& x, const Ref<const MatrixXi>& face_topo, double grid_size, VectorXd& grid_vertices, MatrixXi& grid_topo, MatrixXi& grid_edge_topo, MatrixXi& grid_face_topo, VectorXi& vertices_grid_id, MatrixXd& tri_coefs) = 0;
};

// A very coarse voxelizer
class SimpleMeshVoxelizer : public MeshVoxelizer {
public:
	void Voxelize(const Ref<const VectorXd> &x, const Ref<const MatrixXi> &face_topo, double grid_size, VectorXd &grid_vertices, MatrixXi &grid_topo, MatrixXi &grid_edge_topo, MatrixXi &grid_face_topo, VectorXi &vertices_grid_id, MatrixXd &tri_coefs) override;
};