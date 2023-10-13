// #include "GridData.hpp"
// #include "Voxelizer/Voxelizer.hpp"
// #include "spdlog/spdlog.h"

// GridData::GridData(const json& config)
// 	: GridData(config["filename"], config["proxy-IFN"], config["unit-length-stiffness"], config["unit-length-diag-stiffness"], config["return-stiffness"], config["grid-size"], config["grid-density"]) {}

// GridData::GridData(
// 	const std::string& filename,
// 	int proxy_IFN,
// 	double unit_length_stiffness,
// 	double unit_length_diag_stiffness,
// 	double unit_ret_stiffness,
// 	double grid_size,
// 	double grid_density
// ) : MassSpringData(VectorXd(0), MatrixXi(0, 3), 0, 0),
// 	_proxy(new DynamicSampledObjectData(
// 		(FileIOHelper::ReadMesh(filename, true), _the_vertices),
// 		(FileIOHelper::ReadMesh(filename, true), _the_topo),
// 		0, proxy_IFN
// 	)),
// 	_grid_size(grid_size),
// 	_unit_length_stiffness(unit_length_stiffness),
// 	_unit_length_diag_stiffness(unit_length_diag_stiffness),
// 	_unit_ret_stiffness(unit_ret_stiffness),
// 	_grid_density(grid_density) {
// 	Voxelize();
// }

// GridData::GridData(
// 	const VectorXd& proxy_x,
// 	const MatrixXi& proxy_topo,
// 	int proxy_IFN,
// 	double unit_length_stiffness,
// 	double unit_length_diag_stiffness,
// 	double unit_ret_stiffness,
// 	double grid_size,
// 	double grid_density
// ) : MassSpringData(VectorXd(0), MatrixXi(0, 3), 0, 0),
// 	_proxy(new DynamicSampledObjectData(proxy_x, proxy_topo, 0, proxy_IFN)),
// 	_grid_size(grid_size),
// 	_unit_length_stiffness(unit_length_stiffness),
// 	_unit_length_diag_stiffness(unit_length_diag_stiffness),
// 	_unit_ret_stiffness(unit_ret_stiffness),
// 	_grid_density(grid_density) {
// 	Voxelize();
// }

// void GridData::AddFace() {
// 	_proxy->AddFace();
// }

// void GridData::AddFace(const Vector3d& position) {
// 	_proxy->AddFace(position);
// }

// void GridData::Voxelize() {
// 	_stiffness = _grid_size * _unit_length_stiffness;
// 	_diag_stiffness = _grid_size * _unit_length_diag_stiffness;
// 	_ret_stiffness = _grid_size * _grid_size * _grid_size * _unit_ret_stiffness;
// 	SimpleMeshVoxelizer voxelizer;
// 	voxelizer.Voxelize(
// 		_proxy->_x, _proxy->_face_topo, _grid_size,
// 		_x, _grid_topo, _edge_topo, _face_topo,
// 		_vertices_grid_id, _trilinear_coef
// 	);
// 	spdlog::info("Voxelization finished, #grid = {}, #vertices = {}", _grid_topo.rows(), _x.size() / 3);

// 	_dof = _x.size();
// 	_v = VectorXd::Zero(_dof);
// 	_num_points = _dof / 3;

// 	_mass = VectorXd::Zero(_num_points);

// 	const double single_mass_increment = _grid_density * _grid_size * _grid_size * _grid_size / 8;
// 	const int num_grids = _grid_topo.rows();

// 	_num_edges = _edge_topo.rows();
// 	_start_diag_edges = _num_edges;
// 	_edge_topo.conservativeResize(_num_edges + 4 * num_grids, 2);
// 	_rest_length.resize(_num_edges + 4 * num_grids);
// 	_rest_length.head(_num_edges).setConstant(_grid_size);
// 	_rest_length.tail(4 * num_grids).setConstant(_grid_size * std::sqrt(3));
// 	for (int i = 0; i < num_grids; i++) {
// 		const auto& indices = _grid_topo.row(i);
// 		for (int id = 0; id < 8; id++) {
// 			_mass(indices[id]) += single_mass_increment;
// 		}
// 		_edge_topo.row(_num_edges++) << indices(0), indices(7);
// 		_edge_topo.row(_num_edges++) << indices(1), indices(6);
// 		_edge_topo.row(_num_edges++) << indices(2), indices(5);
// 		_edge_topo.row(_num_edges++) << indices(3), indices(4);
// 	}

// 	_num_faces = _face_topo.rows();
// 	_total_mass = _mass.sum();

// 	_x_rest = _x;
// 	_proxy->_v = VectorXd::Zero(_proxy->_x.size());

// 	_topo_changed = true;
// 	_bb_topo_changed = true;

// 	double max_z = _x(2);
// 	for (int i = 0, i3 = 2; i < _num_points; i++, i3 += 3) {
// 		max_z = std::max(max_z, _x(i3));
// 	}

// 	_fix_points.resize(_num_points);
// 	for (int i = 0; i < _num_points; i++) {
// 		_fix_points[i] = (_x(3 * i + 2) == max_z);
// 	}
// }

// GridData::~GridData() {
// 	delete _proxy;
// }

// #include <iostream>

// void GridData::UpdateProxyPosition() {
// 	for (int i = 0, i3 = 0; i < _proxy->_num_points; i++, i3 += 3) {
// 		const auto& tri_coef = _trilinear_coef.row(i);
// 		const auto& indices = _grid_topo.row(_vertices_grid_id(i));
// 		Vector3d position = Vector3d::Zero();
// 		for (int id = 0; id < 8; id++) {
// 			position += _x.segment<3>(indices(id) * 3)
// 				* ((id & 1) ? tri_coef[0] : (1 - tri_coef[0]))
// 				* ((id & 2) ? tri_coef[1] : (1 - tri_coef[1]))
// 				* ((id & 4) ? tri_coef[2] : (1 - tri_coef[2]));
// 		}
// 		_proxy->_x.segment<3>(i3) = position;
// 	}

// 	#define CHECK(i, j, k, l)\
// 		if ((v[l] - v[i]).dot((v[j] - v[i]).cross(v[k] - v[i])) < 0) {\
// 			inverted = true;\
// 		}
// 	const int num_grids = _grid_topo.rows();
// 	for (int i = 0; i < num_grids; i++) {
// 		Vector3d v[8];
// 		for (int id = 0; id < 8; id++) {
// 			v[id] = _x.segment<3>(_grid_topo(i, id) * 3);
// 		}
// 		bool inverted = false;
// 		CHECK(0, 1, 2, 4)
// 		CHECK(1, 3, 0, 5)
// 		CHECK(2, 0, 3, 6)
// 		CHECK(3, 2, 1, 7)
// 		CHECK(4, 0, 6, 5)
// 		CHECK(5, 1, 4, 7)
// 		CHECK(6, 4, 2, 7)
// 		CHECK(7, 5, 6, 3)
// 		if (inverted) {
// 			std::cerr << "Grid inversion detected" << std::endl;
// 		}
// 	}

// 	#undef CHECK
// }