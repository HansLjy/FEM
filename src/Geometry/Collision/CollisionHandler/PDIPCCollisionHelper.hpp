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

class PDIPCCollisionHandler {
public:
	explicit PDIPCCollisionHandler(const json& config) : PDIPCCollisionHandler(config["kappa"], config["d-hat"]) {}
	PDIPCCollisionHandler(double kappa, double d_hat) : _kappa(kappa), _d_hat(d_hat) {}

	void ClearConstraintSet();
	
	void AddCollisionPairs(
		const std::vector<MassedCollisionInterface>& objs,
		const std::vector<int>& offsets,
		const std::vector<PrimitivePair>& constraint_set,
		const std::vector<double> & local_tois,
		const double global_toi
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

protected:
	double _kappa;
	double _d_hat;

	struct ProjectionInfo {
		ProjectionInfo(int obj_id, int vertex_id, double stiffness, Vector3d target_position) :
			_obj_id(obj_id), _vertex_id(vertex_id), _stiffness(stiffness), _target_position(target_position) {}

		int _obj_id;
		int _vertex_id;
		double _stiffness;
		Vector3d _target_position;
	};
	
	std::vector<ProjectionInfo> _projection_infos;
	InterfaceContainer<MassedCollisionInterface> _obj_container;
	
};