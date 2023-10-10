#pragma once

#include "Optimizer/Optimizer.h"
#include "Assembler/Assembler.hpp"
#include "TimeStepper.hpp"

class IncrementalPotentialTimeStepper : public TimeStepper {
public:
	explicit IncrementalPotentialTimeStepper(const json& config);

	void BindObjects(
		const typename std::vector<Object>::const_iterator &begin,
		const typename std::vector<Object>::const_iterator &end
	) override;
	
	void Step(double h) override;

	~IncrementalPotentialTimeStepper();

private:
	bool _damping_enabled;
	double _rayleigh_coef_mass = 0;
	double _rayleigh_coef_stiffness = 0;
	Optimizer* _optimizer;

	CoordinateAssembler _coord_assembler;
	MassAssembler _mass_assembler;
	EnergyAssembler _energy_assembler;
	ExternalForceAssembler _external_force_assembler;
};

template<class T>
bool RegisterForIP(const std::string& type) {
	RegisterForCaster<Coordinated, T>(type);
	RegisterForCaster<Massed, T>(type);
	RegisterForCaster<Energied, T>(type);
	RegisterForCaster<ExternalForced, T>(type);
	return true;
}