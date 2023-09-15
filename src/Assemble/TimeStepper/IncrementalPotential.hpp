#pragma once

#include "Optimizer/Optimizer.h"
#include "Assembler/Assembler.hpp"
#include "TimeStepper.hpp"

class IncrementalPotentialTimeStepper : public TimeStepper {
public:
	explicit IncrementalPotentialTimeStepper(const json& config);

	void Step(double h) override;

	~IncrementalPotentialTimeStepper();

private:
	bool _damping_enabled;
	double _rayleigh_coef_mass = 0;
	double _rayleigh_coef_stiffness = 0;
	Optimizer* _optimizer;
	Assembler* _assembler;
};
