#pragma once
#include "MassSpringData.hpp"

struct GridData : public MassSpringData {
public:
	explicit GridData(const json& config);
	GridData(const GridData& grid_data) = delete;
	GridData(const std::string& filename, int proxy_IFN, double unit_length_stiffness, double unit_length_diag_stiffness, double unit_ret_stiffness, double grid_size, double grid_density);
	GridData(const VectorXd& proxy_x, const MatrixXi& proxy_topo, int proxy_IFN, double unit_length_stiffness, double unit_length_diag_stiffness, double unit_ret_stiffness, double grid_size, double grid_density);
	void Update();
	void UpdateProxyPosition();
	void AddFace();
	void AddFace(const Vector3d& position);

	virtual ~GridData();
	DynamicSampledObjectData* _proxy;
	double _grid_size;
	double _unit_length_stiffness; // stiffness for a spring of unit length
	double _diag_stiffness;
	double _unit_length_diag_stiffness;
	double _ret_stiffness;
	double _unit_ret_stiffness;
	double _grid_density;
	bool _topo_changed = false;
	bool _bb_topo_changed = false;
	int _start_diag_edges;
	MatrixXi _grid_topo;
	MatrixXd _trilinear_coef;
	VectorXi _vertices_grid_id;
	std::vector<bool> _fix_points;

};

