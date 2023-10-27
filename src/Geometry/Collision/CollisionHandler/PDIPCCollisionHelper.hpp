#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "BlockMatrix.h"
#include "Collision/CollisionInfo.hpp"
#include "Collision/CollisionInterface.hpp"
#include "Object.hpp"

namespace PDIPCCollisionUtility {
	void GetVertexFaceRebounce(
		const Vector3d& vertex,
		const Vector3d& face1, const Vector3d& face2, const Vector3d& face3,
		const Vector3d& vertex_velocity,
		const Vector3d& face_velocity1, const Vector3d& face_velocity2, const Vector3d& face_velocity3,
		const double vertex_mass,
		const double face_mass1, const double face_mass2, const double face_mass3,
		const double toi,
		const double d_hat,
		Vector3d& vertex_after,
		Vector3d& face_after1, Vector3d& face_after2, Vector3d& face_after3
	);

	void GetEdgeEdgeRebounce(
		const Vector3d& edge11, const Vector3d& edge12,
		const Vector3d& edge21, const Vector3d& edge22,
		const Vector3d& edge_velocity11, const Vector3d& edge_velocity12,
		const Vector3d& edge_velocity21, const Vector3d& edge_velocity22,
		const double edge_mass11, const double edge_mass12,
		const double edge_mass21, const double edge_mass22,
		const double toi,
		const double d_hat,
		Vector3d& edge_after11, Vector3d& edge_after12,
		Vector3d& edge_after21, Vector3d& edge_after22
	);

	// The normal points from primitive 1 to primitive 2
	void GetPointPointRebounce(double delta_v, double mass1, double mass2, double& v_after1, double& v_after2);	
	double GetStiffness(double kappa, double d_hat, double distance);
}

class PDIPCCollisionHandler {
public:
	explicit PDIPCCollisionHandler(const json& config) : PDIPCCollisionHandler(
		config["kappa"], config["d-hat"], config["stiffness-blending"]
	) {}
	PDIPCCollisionHandler(double kappa, double d_hat, double stiffness_blending) :
		_kappa(kappa), _d_hat(d_hat), _stiffness_blending(stiffness_blending) {}

	void ClearBarrierSet();

	/**
	 * @param barrier_set The set of primitive pairs with distance < d_hat
	 */
	void AddBarrierPairs(
		const std::vector<CollisionInterface>& objs,
		const std::vector<int>& offsets,
		const std::vector<PrimitivePair>& barrier_candidate_set,
		const double global_toi
	);
	
	// Compute collision shape before calling this function
	double GetBarrierEnergy(
		const std::vector<CollisionInterface>& objs
	) const;

	void BarrierLocalProject(
		const std::vector<CollisionInterface>& objs,
		const std::vector<int>& offsets,
		Ref<VectorXd> y
	) const;
	
	void GetBarrierGlobalMatrix(
		const std::vector<CollisionInterface> &objs,
		const std::vector<int>& offsets,
		COO& coo, int x_offset, int y_offset
	) const;
	
	SparseMatrixXd GetBarrierGlobalMatrix(
		const std::vector<CollisionInterface> &objs,
		const std::vector<int>& offsets,
		int total_dof
	) const;

// protected:
	double _kappa;
	double _d_hat;
	double _stiffness_blending;

	struct BarrierInfo : public PrimitivePair {
		Vector4d _barycentric_coords;
		Vector3d _normal;
		double _stiffness;
	};

	std::vector<BarrierInfo> _barrier_infos;
};