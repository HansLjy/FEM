#pragma once

#include "TimeStepper.hpp"
#include "Collision/CollisionHandler/PDIPCCollisionHelper.hpp"
#include "Collision/CollisionUtil/Culling/CCDCulling.hpp"
#include "Collision/CollisionUtil/TOIEstimator/TOIEstimator.hpp"
#include "Collision/CollisionUtil/Assembler/CollisionAssembler.hpp"
#include "Assembler/Assembler.hpp"
#include "Assembler/PDAssembler.hpp"

class PDIPC : public TimeStepper {
public:
	explicit PDIPC(const json& config);
	void BindSystem(const json &config) override;

	void BindObjects(
        const typename std::vector<Object>::const_iterator &begin,
        const typename std::vector<Object>::const_iterator &end
    );

	void Step(double h) override;

	const std::vector<Renderable> & GetRenderObjects() const override;

	~PDIPC() {
		delete _culling;
	}

protected:
	double _outer_tolerance;
	double _inner_tolerance;

	std::vector<int> _offsets;
	std::vector<Renderable> _render_objs;
	std::vector<CollisionInterface> _collision_objs;
	std::vector<MassedCollisionInterface> _massed_collision_objs;

	CCDCulling* _culling;

	CoordinateAssembler _coord_assembler;
	MassAssembler _mass_assembler;
	ExternalForceAssembler _ext_force_assembler;

	PDAssembler _pd_assembler;
	PDIPCCollisionHandler _pd_ipc_collision_handler;
	CollisionAssembler _collision_assembler;
	TOIEstimator _toi_estimator;

};

namespace ObjectRegistration {
	template<class T>
	bool RegisterForPDIPC(const std::string& type) {
		CasterRegistration::RegisterForCaster<Renderable, T>(type);
		CasterRegistration::RegisterForCaster<CollisionInterface, T>(type);
		CasterRegistration::RegisterForCaster<MassedCollisionInterface, T>(type);
		CasterRegistration::RegisterForCaster<MassedCollisionInterface, T>(type);
		return true;
	}
}