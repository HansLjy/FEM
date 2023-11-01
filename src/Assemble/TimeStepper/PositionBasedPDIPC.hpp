#pragma once

#include "TimeStepper.hpp"
#include "Collision/CollisionHandler/PositionBasedPDIPCCollisionHelper.hpp"
#include "Collision/CollisionUtil/Culling/CCDCulling.hpp"
#include "Collision/CollisionUtil/TOIEstimator/TOIEstimator.hpp"
#include "Collision/CollisionUtil/Assembler/CollisionAssembler.hpp"
#include "Assembler/Assembler.hpp"
#include "Assembler/PDAssembler.hpp"

class PositionBasedPDIPC : public TimeStepper {
public:
	static PositionBasedPDIPC* CreateFromConfig(const json& config);

	PositionBasedPDIPC(
		double outer_tolerance, double inner_tolerance,
		int outer_max_iterations, int inner_max_iterations,
		PositionBasedPDIPCCollisionHandler&& collision_handler,
		TOIEstimator&& toi_estimator,
		CCDCulling* culling
	);

	void BindSystem(const json &config) override;

	void BindObjects(
        const typename std::vector<Object>::const_iterator &begin,
        const typename std::vector<Object>::const_iterator &end
    );

	void Step(double h) override;

	void InnerIteration(const SparseMatrixXd& lhs_out, const VectorXd& rhs_out, VectorXd& x) const;
	double GetTotalEnergy(const VectorXd& x, const SparseMatrixXd& M_h2, const VectorXd& x_hat);

	const std::vector<Renderable> & GetRenderObjects() const override;

	~PositionBasedPDIPC() {
		delete _culling;
	}

protected:
	double _outer_tolerance;
	double _inner_tolerance;

	int _outer_max_itrs;
	int _inner_max_itrs;

	int _total_dof;
	std::vector<int> _offsets;
	std::vector<Object> _objs;
	std::vector<Renderable> _render_objs;
	std::vector<CollisionInterface> _collision_objs;
	std::vector<MassedCollisionInterface> _massed_collision_objs;
	std::vector<PDObject> _pd_objs;


	CoordinateAssembler _coord_assembler;
	MassAssembler _mass_assembler;
	ExternalForceAssembler _ext_force_assembler;

	PDAssembler _pd_assembler;
	PositionBasedPDIPCCollisionHandler _pb_pd_ipc_collision_handler;
	CollisionAssembler _collision_assembler;
	TOIEstimator _toi_estimator;

	CCDCulling* _culling;
};

namespace ObjectRegistration {
	template<class T>
	bool RegisterForPositionBasedPDIPC(const std::string& type) {
		CasterRegistration::RegisterForCaster<Renderable, T>(type);
		CasterRegistration::RegisterForCaster<Coordinated, T>(type);
		CasterRegistration::RegisterForCaster<Massed, T>(type);
		CasterRegistration::RegisterForCaster<ExternalForced, T>(type);
		CasterRegistration::RegisterForCaster<PDObject, T>(type);
		CasterRegistration::RegisterForCaster<CollisionInterface, T>(type);
		CasterRegistration::RegisterForCaster<MassedCollisionInterface, T>(type);
		return true;
	}
}