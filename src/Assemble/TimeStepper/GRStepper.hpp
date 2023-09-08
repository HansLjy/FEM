#pragma once

#include "IPC.hpp"
#include "System/GRSystem.hpp"

class GeometryReconstructStepper : public TimeStepper {
public:
	explicit GeometryReconstructStepper(const json& config);

	void Bind(System *system) override;
	void Step(double h) override;

	~GeometryReconstructStepper();

protected:
	double _eps;
	double _stuck_tolerance;
	IPC* _ipc = nullptr;
	GeometryReconstructSystem* _gr_system = nullptr;

	bool IsSteady(const Vector9d& a);
	void UpdateTractionForce();
};