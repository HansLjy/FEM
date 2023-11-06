#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "BlockMatrix.h"
#include "Collision/CollisionInfo.hpp"
#include "Object.hpp"

CONCEPT_MODEL_IDIOM_BEGIN(MassedCollisionInterface)
    ADD_INTERFACE_FUNCTION(int GetDOF() const, GetDOF())
    ADD_INTERFACE_FUNCTION(const BlockVector& GetCollisionVertexDerivative(int idx) const, GetCollisionVertexDerivative(idx))
    ADD_INTERFACE_FUNCTION(void ComputeCollisionVertex(const Ref<const VectorXd>& x) const, ComputeCollisionVertex(x))
    ADD_INTERFACE_FUNCTION(void ComputeCollisionVertexVelocity(const Ref<const VectorXd>& v) const, ComputeCollisionVertexVelocity(v))
    ADD_INTERFACE_FUNCTION(Vector3d GetCollisionVertexVelocity(int idx) const, GetCollisionVertexVelocity(idx))
    ADD_INTERFACE_FUNCTION(const MatrixXd& GetCollisionVertices() const, GetCollisionVertices())
    ADD_INTERFACE_FUNCTION(const MatrixXi& GetCollisionEdgeTopo() const, GetCollisionEdgeTopo())
    ADD_INTERFACE_FUNCTION(const MatrixXi& GetCollisionFaceTopo() const, GetCollisionFaceTopo())
	ADD_INTERFACE_FUNCTION(double GetCollisionVertexMass(int idx) const, GetCollisionVertexMass(idx))
CONCEPT_MODEL_IDIOM_CONCEPT
    ADD_CONCEPT_FUNCTION(int GetDOF() const)
    ADD_CONCEPT_FUNCTION(const BlockVector& GetCollisionVertexDerivative(int idx) const)
    ADD_CONCEPT_FUNCTION(void ComputeCollisionVertex(const Ref<const VectorXd>& x) const)
    ADD_CONCEPT_FUNCTION(void ComputeCollisionVertexVelocity(const Ref<const VectorXd>& v) const)
    ADD_CONCEPT_FUNCTION(Vector3d GetCollisionVertexVelocity(int idx) const)
    ADD_CONCEPT_FUNCTION(const MatrixXd& GetCollisionVertices() const)
    ADD_CONCEPT_FUNCTION(const MatrixXi& GetCollisionEdgeTopo() const)
    ADD_CONCEPT_FUNCTION(const MatrixXi& GetCollisionFaceTopo() const)
	ADD_CONCEPT_FUNCTION(double GetCollisionVertexMass(int idx) const)
CONCEPT_MODEL_IDIOM_MODEL
    ADD_MODEL_FUNCTION(int GetDOF() const, GetDOF())
    ADD_MODEL_FUNCTION(const BlockVector& GetCollisionVertexDerivative(int idx) const, GetCollisionVertexDerivative(idx))
    ADD_MODEL_FUNCTION(void ComputeCollisionVertex(const Ref<const VectorXd>& x) const, ComputeCollisionVertex(x))
    ADD_MODEL_FUNCTION(void ComputeCollisionVertexVelocity(const Ref<const VectorXd>& v) const, ComputeCollisionVertexVelocity(v))
    ADD_MODEL_FUNCTION(Vector3d GetCollisionVertexVelocity(int idx) const, GetCollisionVertexVelocity(idx))
    ADD_MODEL_FUNCTION(const MatrixXd& GetCollisionVertices() const, GetCollisionVertices())
    ADD_MODEL_FUNCTION(const MatrixXi& GetCollisionEdgeTopo() const, GetCollisionEdgeTopo())
    ADD_MODEL_FUNCTION(const MatrixXi& GetCollisionFaceTopo() const, GetCollisionFaceTopo())
	ADD_MODEL_FUNCTION(double GetCollisionVertexMass(int idx) const, GetCollisionVertexMass(idx))
CONCEPT_MODEL_IDIOM_END

namespace PositionBasedPDIPCCollisionUtility {
	void GetVertexFaceRebounce(
		const Vector3d &vertex,
		const Vector3d &face1, const Vector3d &face2, const Vector3d &face3,
		const Vector3d &vertex_velocity,
		const Vector3d &face_velocity1, const Vector3d &face_velocity2, const Vector3d &face_velocity3,
		const double vertex_mass,
		const double face_mass1, const double face_mass2, const double face_mass3,
		const double local_toi,
		const double d_hat,
		Vector3d &vertex_after,
		Vector3d &face_after1, Vector3d &face_after2, Vector3d &face_after3
	);

	void GetEdgeEdgeRebounce(
		const Vector3d &edge11, const Vector3d &edge12,
		const Vector3d &edge21, const Vector3d &edge22,
		const Vector3d &edge_velocity11, const Vector3d &edge_velocity12,
		const Vector3d &edge_velocity21, const Vector3d &edge_velocity22,
		const double edge_mass11, const double edge_mass12,
		const double edge_mass21, const double edge_mass22,
		const double local_toi,
		const double d_hat,
		Vector3d &edge_after11, Vector3d &edge_after12,
		Vector3d &edge_after21, Vector3d &edge_after22
	);

	double GetStiffness(double kappa, double d_hat2, double distance2, double min_stffness);
}

class PositionBasedPDIPCCollisionHandler {
public:
	explicit PositionBasedPDIPCCollisionHandler(const json& config) : PositionBasedPDIPCCollisionHandler(
		config["kappa"], config["d-hat"], config["stiffness-blending"], config["velocity-damping"]
	) {}
	PositionBasedPDIPCCollisionHandler(double kappa, double d_hat, double stiffness_blending, double velocity_damping) :
		_kappa(kappa), _d_hat(d_hat), _stiffness_blending(stiffness_blending), _velocity_damping(velocity_damping){}

	void ClearConstraintSet();
	
	void AddCollisionPairs(
		const std::vector<MassedCollisionInterface>& objs,
		const std::vector<int>& offsets,
		const std::vector<PrimitivePair>& constraint_set,
		const std::vector<double> & local_tois,
		const double global_toi,
		double dt
	);

	double GetBarrierEnergy(
		const std::vector<MassedCollisionInterface>& objs
	) const;

	void BarrierLocalProject(
		const std::vector<MassedCollisionInterface>& objs,
		const std::vector<int>& offsets,
		Ref<VectorXd> y
	) const;
	
	void GetBarrierGlobalMatrix(
		const std::vector<MassedCollisionInterface> &objs,
		const std::vector<int>& offsets,
		COO& coo, int x_offset, int y_offset
	) const;
	
	void GetBarrierGlobalMatrix(
		const std::vector<MassedCollisionInterface> &objs,
		const std::vector<int>& offsets,
		int total_dof,
		SparseMatrixXd& global_matrix
	) const;

// protected:
	double _kappa;
	double _d_hat;
	double _stiffness_blending;
	double _velocity_damping;

	struct ProjectionInfo {
		ProjectionInfo(int obj_id, int vertex_id, double stiffness, Vector3d target_position) :
			_obj_id(obj_id), _vertex_id(vertex_id), _stiffness(stiffness), _target_position(target_position) {}

		int _obj_id;
		int _vertex_id;
		double _stiffness;
		Vector3d _target_position;
	};
	
	std::vector<ProjectionInfo> _projection_infos;
	std::vector<PrimitivePair> _primitive_pairs;					// every primitive pair cooresponds to 4 projection infos
	InterfaceContainer<MassedCollisionInterface> _obj_container;
	
};