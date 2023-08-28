#pragma once
#include "MassSpringData.hpp"

struct CoarseGridBasedData : public MassSpringData {
public:
	explicit CoarseGridBasedData(const json& config);
	CoarseGridBasedData(const std::string& filename, double proxy_density, int proxy_IFN, double stiffness, double grid_size);
	CoarseGridBasedData(const VectorXd& proxy_x, const MatrixXi& proxy_topo, double proxy_density, int proxy_IFN, double stiffness, double grid_size);
	void Update();
	void UpdateProxyPosition();
	void AddFace(int id1, int id2, int id3);
	void AddFace(int id1, int id2, const Vector3d& position);

	virtual ~CoarseGridBasedData();
	DynamicSampledObjectData* _proxy;
	double _grid_size;
	bool _topo_changed = false;
	bool _bb_topo_changed = false;
	MatrixXi _grid_topo;
	MatrixXd _trilinear_coef;
	VectorXi _vertices_grid_id;

};

