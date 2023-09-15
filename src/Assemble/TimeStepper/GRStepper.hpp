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
	int _nearly_stop_successive_itrs = 0;
	int _stuck_tolerance_itrs;
	double _stuck_tolerance_velocity;
	double _stuck_tolerance_acceleration;
	IPC* _ipc = nullptr;
	GeometryReconstructSystem* _gr_system = nullptr;
};