#include "GRStepper.hpp"
#include "FileIO.hpp"

GeometryReconstructStepper::GeometryReconstructStepper(const json& config)
	: _ipc(new IPC(config["ipc"])), _eps(config["eps"]), _stuck_tolerance(config["stuck-tolerance"]){}

void GeometryReconstructStepper::Bind(System *system) {
	TimeStepper::Bind(system);
	_ipc->Bind(_system);
	_gr_system = dynamic_cast<GeometryReconstructSystem*>(system);
	assert(_gr_system != nullptr);
}

void GeometryReconstructStepper::Step(double h) {
	Vector9d v_before = _gr_system->_new_triangle->_v;
	_ipc->Step(h);
	Vector9d a = (_gr_system->_new_triangle->_v - v_before) / h;

	if (IsSteady(a)) {
		if (_gr_system->IsNear(_eps)) {
			// Successfully glued
		} else {
			// stucked, need to recompute tracktion force
			UpdateTractionForce();
		}
	}
}

bool GeometryReconstructStepper::IsSteady(const Vector9d& a) {
	return (a.lpNorm<1>() < _stuck_tolerance && _gr_system->_new_triangle->_v.lpNorm<1>() < _stuck_tolerance);
}

void GeometryReconstructStepper::UpdateTractionForce() {
	const auto& constraint_set = _ipc->_helper->_constraint_set;
	for (const auto& constraint_pair : constraint_set) {
		if (constraint_pair._obj_id1 != constraint_pair._obj_id2) {
			// Non self-collision
			// TODO:
		}
	}
}

GeometryReconstructStepper::~GeometryReconstructStepper() {
	delete _ipc;
}