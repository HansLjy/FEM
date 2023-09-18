#pragma once
#include "JsonUtil.h"
#include "EigenAll.h"
#include "Objects/Grid.hpp"
#include "Objects/Triangle.hpp"

class GeometryReconstructSystem;
class CustomGrid : public Grid {
public:
	CustomGrid(
		const VectorXd& proxy_x, const MatrixXi& proxy_topo, int proxy_IFN,
		double unit_length_stiffness, double unit_length_diag_stiffness, double unit_ret_stiffness,
		double grid_size, double grid_density,
		bool have_bounding_box
	) : Grid(GridData(proxy_x, proxy_topo, proxy_IFN, unit_length_stiffness, unit_length_diag_stiffness, unit_ret_stiffness, grid_size, grid_density), GridEnergyModel(), GridShape(have_bounding_box), GridCollisionShape()) {}

	Vector3d GetVertex(int id, const Ref<const VectorXd>& x) const;
	Vector3d GetVertex(int id) const;
	const BlockVector& GetVertexDerivative(int id) const;

	friend GeometryReconstructSystem;

private:
	void AddDanglingVertex(
		const Vector3d& dangling_vertex,
		int glue_id1, int glue_id2,
		double rest_length1, double rest_length2
	);

	void MergeDanglingVertex(bool is_new_vertex);

	void AddFace();
	void AddFace(const Vector3d& position);

	// dangling vertex: a vertex connected to the grid but is not under 
	//					the control of the grid
	bool _has_dangling_vertex;
	Vector3d _dangling_vertex;
	int _glue_id[2];
	double _rest_length[2];
};


class CustomTriangle : public Triangle {
public:
	CustomTriangle(const Vector9d& x, const Vector9d& x_rest, double density, double stiffness, double ret_stiffness) : Triangle(TriangleData(x, x_rest, density, stiffness, ret_stiffness), MassSpringEnergyModel(), SampledRenderShape(false, false), SampledCollisionShape()) {}

	Vector3d GetVertex(int id, const Ref<const VectorXd>& x) const;
	Vector3d GetVertex(int id) const;
	const BlockVector& GetVertexDerivative(int id) const;
};

namespace GRSystemStatusCode {
	const int kStuck = 1;
};

struct GeometryReconstructSystem {
	enum class SystemStatus {
		kFlying,		// flying to the destination
		kGlueing,		// glue the final two points together
		kCrawling,		// crawling along geodesic
		kTurning,		// The topology has been fixed, only geometric
						// constraint(dihedral angles) remain to be fixed
	} _status;

	explicit GeometryReconstructSystem(const json& config);
	GeometryReconstructSystem(const GeometryReconstructSystem& rhs) = delete;
	~GeometryReconstructSystem();

	// Fix two points of the triangle to the grid
	void AttachTriangle();

	// Merge the triangle into the reconstructed mesh
	void MergeTriangle();

	// Add new triangle into the system
	void GetNewTriangle();

	void UpdateLocalTarget(int face_id, int side);

	std::vector<Object*> _objs;
	CustomGrid* _reconstructed_mesh = nullptr;
	CustomTriangle* _new_triangle = nullptr;

	double _triangle_fly_stiffness;
	double _triangle_glue_stiffness;
	double _triangle_crawl_stiffness;

	double _crawling_distance;

	int _attach_vertex_id[3];

	int _global_target_id;

	// Local target: when the new triangle gets stuck flying to the global
	//               target (the face that has topology connection with it),
	//               we should pull it to the local target instead, which
	//				 is the next triangle to the current collision face in
	//				 the geodesic path to the global target
	int _local_target_id;					// face id for local target
	int _local_target_orientation;			// face orientation for local target
	Vector3d _local_target_position;		// local target position


protected:
	std::string _obj_name;

	MatrixX3i _attach_vertex_ids;	// [i, j]: the id of the jth vertex of the ith triangle
	VectorXi _father_face_id;		// [i]: the triangle which face[i] is glued to

	std::vector<int> _prev_face;								// the path of faces to a certain element
	std::vector<bool> _inverted_face;							// whether face[i] and face[_prev_face[i]] has different position
	std::vector<std::vector<int>> _neighbor_faces;
	std::vector<std::vector<bool>> _is_neighbor_face_inverted;	// [i, j]: whether face[i] and the jth neighbor of face[i] has different normal direction

	int _total_faces;
	int _num_reconstructed_faces;
	double _triangle_density;
	double _triangle_stiffness;

	// reorder the vertices
	void Preprocessing(VectorXd& vertices, MatrixXi& topo);
	void CalculateReturnPath(int source_face_id);
};