#pragma once

#include "Assembler/Assembler.hpp"
#include "Assembler/PDAssembler.hpp"
#include "Collision/CollisionHandler/PDCollisionHandler.hpp"
#include "TimeStepper.hpp"

class ProjectiveDynamics : public TimeStepper {
public:
    ProjectiveDynamics(const json& config);

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

	std::vector<Object> _objs;
	std::vector<Object> _colliders;

	std::vector<Renderable> _render_objects;

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
}
