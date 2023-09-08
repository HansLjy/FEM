#pragma once
#include "JsonUtil.h"
#include "EigenAll.h"
#include "Objects/Grid.hpp"
#include "Objects/Triangle.hpp"

class CustomGrid : public Grid {
public:
	CustomGrid(const VectorXd& proxy_x, const MatrixXi& proxy_topo, int proxy_IFN, double unit_length_stiffness, double unit_length_diag_stiffness, double unit_ret_stiffness, double grid_size, double grid_density, bool have_bounding_box) : Grid(GridData(proxy_x, proxy_topo, proxy_IFN, unit_length_stiffness, unit_length_diag_stiffness, unit_ret_stiffness, grid_size, grid_density), GridEnergyModel(), GridShape(have_bounding_box), NullCollisionShape()) {}
};


class CustomTriangle : public Triangle {
public:
	CustomTriangle(const Vector9d& x, const Vector9d& x_rest, double density, double stiffness, double ret_stiffness) : Triangle(TriangleData(x, x_rest, density, stiffness, ret_stiffness), SumOfEnergyModel(MassSpringEnergyModel(), FixtureEnergyModel()), SampledRenderShape(false, false), SampledCollisionShape()) {}
};

struct GeometryReconstructSystem {
	enum class SystemStatus {
		kGlueing,		// glue the final two points together
		kCrawling,		// crawling along geodesic
		kFlying			// flying towards the destination
	};

	explicit GeometryReconstructSystem(const json& config);
	GeometryReconstructSystem(const GeometryReconstructSystem& rhs) = delete;
	~GeometryReconstructSystem();

	// Called upon the finish of current triangle reconstruction
	void Update();

	//<- whether the newly added triangle is near its target position
	bool IsNear(double eps) const;

	void OnStuck();

	std::vector<Object*> _objs;
	CustomGrid* _reconstructed_mesh = nullptr;
	CustomTriangle* _new_triangle = nullptr;

protected:
	MatrixX3i _glue_ids;
	VectorXi _father_face_id;

	std::vector<int> _prev_vertex;							// the path of faces to a certain element
	std::vector<std::vector<int>> _neighbor_faces;

	int _total_faces;
	int _num_reconstructed_faces;
	double _triangle_density;
	double _triangle_stiffness;
	double _triangle_ret_stiffness;

	// reorder the vertices
	void Preprocessing(VectorXd& vertices, MatrixXi& topo);
	void CalculateReturnPath(int source_face_id);
	void GlueTriangle();
	void GetNewTriangle();


};