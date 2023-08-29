#pragma once
#include "MassSpringData.hpp"

struct CoarseGridBasedData : public MassSpringData {
public:
	explicit CoarseGridBasedData(const json& config);
	CoarseGridBasedData(const std::string& filename, int proxy_IFN, double unit_length_stiffness, double unit_ret_stiffness, double grid_size, double grid_density);
	CoarseGridBasedData(const VectorXd& proxy_x, const MatrixXi& proxy_topo, int proxy_IFN, double unit_length_stiffness, double unit_ret_stiffness, double grid_size, double grid_density);
	void Update();
	void UpdateProxyPosition();
	void AddFace(int id1, int id2, int id3);
	void AddFace(int id1, int id2, const Vector3d& position);

	virtual ~CoarseGridBasedData();
	DynamicSampledObjectData* _proxy;
	double _grid_size;
	double _unit_length_stiffness; // stiffness for a spring of unit length
	double _ret_stiffness;
	double _unit_ret_stiffness;
	double _grid_density;
	bool _topo_changed = false;
	bool _bb_topo_changed = false;
	MatrixXi _grid_topo;
	MatrixXd _trilinear_coef;
	VectorXi _vertices_grid_id;

};

