#include "CoarseGridBasedData.hpp"
#include "Voxelizer/Voxelizer.hpp"

CoarseGridBasedData::CoarseGridBasedData(
	const VectorXd& proxy_x,
	const MatrixXi& proxy_topo,
	double proxy_density,
	int proxy_dimension,
	int proxy_IFN,
	double stiffness
) : MassSpringData(VectorXd(0), MatrixXi(0, 3), 0, stiffness),
	_proxy(proxy_x, proxy_density, proxy_dimension, proxy_topo, proxy_IFN) {
	Update();
}

void CoarseGridBasedData::Update() {
	SimpleMeshVoxelizer voxelizer;
	// TODO: adjust grid size
	MatrixXi grid_topo;
	voxelizer.Voxelize(_proxy._x, _proxy._face_topo, 1, _x, grid_topo, _edge_topo, _face_topo);
	_dof = _proxy._x.size();
	_proxy._v = VectorXd::Zero(_proxy._x.size());
	_num_points = _dof / 3;
	_num_edges = _edge_topo.rows();
	_num_faces = _face_topo.rows();
}