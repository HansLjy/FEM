#pragma once

#include "Optimizer/Optimizer.h"
#include "Assembler/Assembler.hpp"
#include "TimeStepper.hpp"

class IncrementalPotentialTimeStepper : public TimeStepper {
public:
	static IncrementalPotentialTimeStepper* CreateFromConfig(const json& config);
	IncrementalPotentialTimeStepper(double damping_enabled, double rayleigh_coef_mass, double rayleigh_coef_stiffness, Optimizer* optimizer);

	void BindSystem(const json &config) override;
	void BindObjects(
		const typename std::vector<Object>::const_iterator &begin,
		const typename std::vector<Object>::const_iterator &end
	);
	
	void Step(double h) override;
	const std::vector<Renderable> & GetRenderObjects() const override;

	~IncrementalPotentialTimeStepper();

private:
	bool _damping_enabled;
	double _rayleigh_coef_mass = 0;
	double _rayleigh_coef_stiffness = 0;
	Optimizer* _optimizer;

	std::vector<Renderable> _render_objects;

	CoordinateAssembler _coord_assembler;
	MassAssembler _mass_assembler;
	EnergyAssembler _energy_assembler;
	ExternalForceAssembler _external_force_assembler;
};

template<class T>
bool RegisterForIP(const std::string& type) {
	CasterRegistration::RegisterForCaster<Coordinated, T>(type);
	CasterRegistration::RegisterForCaster<Massed, T>(type);
	CasterRegistration::RegisterForCaster<Energied, T>(type);
	CasterRegistration::RegisterForCaster<ExternalForced, T>(type);
	return true;
}