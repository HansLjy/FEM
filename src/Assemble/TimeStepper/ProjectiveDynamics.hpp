#pragma once

#include "Assembler/Assembler.hpp"
#include "Assembler/PDAssembler.hpp"
#include "Collision/CollisionHandler/PDCollisionHandler.hpp"
#include "TimeStepper.hpp"

class ProjectiveDynamics : public TimeStepper {
public:
	static ProjectiveDynamics* CreateFromConfig(const json& config);
	ProjectiveDynamics(
		int max_step, double tolerance,
		PDCollisionHandler&& collision_handler
	);

	void BindSystem(const json &config) override;

    void BindObjects(
        const typename std::vector<Object>::const_iterator &begin,
        const typename std::vector<Object>::const_iterator &end
    );

	void BindCollider(
		const typename std::vector<Object>::const_iterator &begin,
        const typename std::vector<Object>::const_iterator &end
	);
    
    void Step(double h) override;
	const std::vector<Renderable> & GetRenderObjects() const override;

protected:
    int _max_step;
    double _conv_tolerance;

    int _total_dof;
	std::vector<Object> _objs;
	std::vector<Object> _colliders;

	std::vector<Renderable> _render_objects;
    std::vector<PDObject> _pd_objects;

    CoordinateAssembler _coord_assembler;
    MassAssembler _mass_assembler;
    ExternalForceAssembler _ext_force_assembler;
    PDAssembler _pd_assembler;
	PDCollisionHandler _pd_collision_handler;
};

namespace ObjectRegistration {
    template<class T>
    bool RegisterForPD(const std::string& type) {
        CasterRegistration::RegisterForCaster<Coordinated, T>(type);
        CasterRegistration::RegisterForCaster<Massed, T>(type);
        CasterRegistration::RegisterForCaster<ExternalForced, T>(type);
        CasterRegistration::RegisterForCaster<PDObject, T>(type);
		CasterRegistration::RegisterForCaster<CollisionInterface, T>(type);
        return true;
    }

    template<class T>
    bool RegisterForPDCollider(const std::string& type) {
        CasterRegistration::RegisterForCaster<Collider, T>(type);
        return true;
    }
}
