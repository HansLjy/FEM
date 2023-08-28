#include "CoarseGridBasedData.hpp"
#include "Voxelizer/Voxelizer.hpp"
#include "spdlog/spdlog.h"

CoarseGridBasedData::CoarseGridBasedData(const json& config)
	: CoarseGridBasedData(config["filename"], config["proxy-density"], config["proxy-IFN"], config["stiffness"], config["grid-size"]) {}

CoarseGridBasedData::CoarseGridBasedData(const std::string& filename, double proxy_density, int proxy_IFN, double stiffness, double _grid_size)
	: MassSpringData(VectorXd(0), MatrixXi(0, 3), 0, stiffness),
	 _proxy(new DynamicSampledObjectData(filename, proxy_density, proxy_IFN)),
	 _grid_size(_grid_size) {
	Update();
}

CoarseGridBasedData::CoarseGridBasedData(
	const VectorXd& proxy_x,
	const MatrixXi& proxy_topo,
	double proxy_density,
	int proxy_IFN,
	double stiffness,
	double grid_size
) : MassSpringData(VectorXd(0), MatrixXi(0, 3), 0, stiffness),
	_proxy(new DynamicSampledObjectData(proxy_x, proxy_density, proxy_topo, proxy_IFN)),
	_grid_size(grid_size) {
	Update();
}

void CoarseGridBasedData::AddFace(int id1, int id2, int id3) {
	_proxy->AddFace(id1, id2, id3);
}

void CoarseGridBasedData::AddFace(int id1, int id2, const Vector3d& position) {
	_proxy->AddFace(id1, id2, position);
}

void CoarseGridBasedData::Update() {
	SimpleMeshVoxelizer voxelizer;
	voxelizer.Voxelize(
		_proxy->_x, _proxy->_face_topo, _grid_size,
		_x, _grid_topo, _edge_topo, _face_topo,
		_vertices_grid_id, _trilinear_coef
	);
	spdlog::info("Voxelization finished, #grid = {}, #vertices = {}", _grid_topo.rows(), _x.size() / 3);
	_dof = _x.size();
	_v = VectorXd::Zero(_dof);
	_num_points = _dof / 3;

	_mass = VectorXd::Zero(_num_points);
	for (int i = 0; i < _proxy->_num_points; i++) {
		const auto& tri_coef = _trilinear_coef.row(i);
		const auto& indices = _grid_topo.row(_vertices_grid_id(i));
		const double single_mass = _proxy->_mass(i);
		for (int id = 0; id < 8; id++) {
			_mass(indices[id]) += single_mass
				* ((id & 1) ? tri_coef[0] : (1 - tri_coef[0]))
				* ((id & 2) ? tri_coef[1] : (1 - tri_coef[1]))
				* ((id & 4) ? tri_coef[2] : (1 - tri_coef[2]));
		}
	}

	_num_edges = _edge_topo.rows();
	_num_faces = _face_topo.rows();
	_total_mass = _mass.sum();

	_x_rest = _x;
	_rest_length = VectorXd::Constant(_num_edges, _grid_size);
	_proxy->_v = VectorXd::Zero(_proxy->_x.size());

	_topo_changed = true;
	_bb_topo_changed = true;
}

CoarseGridBasedData::~CoarseGridBasedData() {
	delete _proxy;
}

void CoarseGridBasedData::UpdateProxyPosition() {
	for (int i = 0, i3 = 0; i < _proxy->_num_points; i++, i3 += 3) {
		const auto& tri_coef = _trilinear_coef.row(i);
		const auto& indices = _grid_topo.row(_vertices_grid_id(i));
		Vector3d position = Vector3d::Zero();
		for (int id = 0; id < 8; id++) {
			position += _x.segment<3>(indices(id) * 3)
				* ((id & 1) ? tri_coef[0] : (1 - tri_coef[0]))
				* ((id & 2) ? tri_coef[1] : (1 - tri_coef[1]))
				* ((id & 4) ? tri_coef[2] : (1 - tri_coef[2]));
		}
		_proxy->_x.segment<3>(i3) = position;
	}
}