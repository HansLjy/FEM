// #pragma once

// #include "TimeStepper.hpp"
// #include "Assembler/Assembler.hpp"
// #include "Collision/IPC/IPCHelper.hpp"

// class IPC : public TimeStepper {
// public:
// 	explicit IPC(const json& config);

// 	void BindObjects(
// 		const typename std::vector<Object>::const_iterator &begin,
// 		const typename std::vector<Object>::const_iterator &end
// 	) override;

// 	void Step(double h) override;
	
// 	int _max_iter;
//     double _tolerance;
// 	CoordinateAssembler _coord_assembler;
// 	MassAssembler _mass_assembler;
// 	EnergyAssembler _energy_assembler;
// 	ExternalForceAssembler _external_force_assembler;

// 	IPCEnergy _ipc_energy;
// 	MaxStepEstimator _max_step_estimator;
// 	ConstraintSetGenerator _constraint_set_generator;
// };
