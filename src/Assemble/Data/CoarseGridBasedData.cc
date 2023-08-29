#include "CoarseGridBasedData.hpp"
#include "Voxelizer/Voxelizer.hpp"
#include "spdlog/spdlog.h"

CoarseGridBasedData::CoarseGridBasedData(const json& config)
	: CoarseGridBasedData(config["filename"], config["proxy-IFN"], config["unit-length-stiffness"], config["return-stiffness"], config["grid-size"], config["grid-density"]) {}

CoarseGridBasedData::CoarseGridBasedData(
	const std::string& filename,
	int proxy_IFN,
	double unit_length_stiffness,
	double unit_ret_stiffness,
	double grid_size,
	double grid_density
) : MassSpringData(VectorXd(0), MatrixXi(0, 3), 0, 0),
	_proxy(new DynamicSampledObjectData(filename, 0, proxy_IFN)),
	_grid_size(grid_size),
	_unit_length_stiffness(unit_length_stiffness),
	_unit_ret_stiffness(unit_ret_stiffness),
	_grid_density(grid_density) {
	Update();
}

CoarseGridBasedData::CoarseGridBasedData(
	const VectorXd& proxy_x,
	const MatrixXi& proxy_topo,
	int proxy_IFN,
	double unit_length_stiffness,
	double unit_ret_stiffness,
	double grid_size,
	double grid_density
) : MassSpringData(VectorXd(0), MatrixXi(0, 3), 0, 0),
	_proxy(new DynamicSampledObjectData(proxy_x, 0, proxy_topo, proxy_IFN)),
	_grid_size(grid_size),
	_unit_length_stiffness(unit_length_stiffness),
	_unit_ret_stiffness(unit_ret_stiffness),
	_grid_density(grid_density) {
	Update();
}

void CoarseGridBasedData::AddFace(int id1, int id2, int id3) {
	_proxy->AddFace(id1, id2, id3);
}

void CoarseGridBasedData::AddFace(int id1, int id2, const Vector3d& position) {
	_proxy->AddFace(id1, id2, position);
}

void CoarseGridBasedData::Update() {
	_stiffness = _grid_size * _unit_length_stiffness;
	_ret_stiffness = _grid_size * _grid_size * _grid_size * _unit_ret_stiffness;
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

	const double single_mass_increment = _grid_density * _grid_size * _grid_size * _grid_size / 8;
	const int num_grids = _grid_topo.rows();

	_num_edges = _edge_topo.rows();
	_edge_topo.conservativeResize(_num_edges + 4 * num_grids, 2);
	_rest_length.resize(_num_edges + 4 * num_grids);
	_rest_length.head(_num_edges).setConstant(_grid_size);
	_rest_length.tail(4 * num_grids).setConstant(_grid_size * std::sqrt(3));
	for (int i = 0; i < num_grids; i++) {
		const auto& indices = _grid_topo.row(i);
		for (int id = 0; id < 8; id++) {
			_mass(indices[id]) += single_mass_increment;
		}
		_edge_topo.row(_num_edges++) << indices(0), indices(7);
		_edge_topo.row(_num_edges++) << indices(1), indices(6);
		_edge_topo.row(_num_edges++) << indices(2), indices(5);
		_edge_topo.row(_num_edges++) << indices(3), indices(4);
	}

	_num_faces = _face_topo.rows();
	_total_mass = _mass.sum();

	_x_rest = _x;
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