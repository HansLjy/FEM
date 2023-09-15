#include "GRStepper.hpp"
#include "FileIO.hpp"

namespace {
	const bool gr_registered = Factory<TimeStepper>::GetInstance()->Register("geometry-reconstruction", [](const json& config){
		return new GeometryReconstructStepper(config);
	});
}

GeometryReconstructStepper::GeometryReconstructStepper(const json& config)
	: _ipc(new IPC(config["ipc"])) {}

void GeometryReconstructStepper::Bind(System *system) {
	TimeStepper::Bind(system);
	_ipc->Bind(_system);
	_gr_system = dynamic_cast<GeometryReconstructSystem*>(system);
	assert(_gr_system != nullptr);
}

void GeometryReconstructStepper::Step(double h) {
	
	Vector9d v_before = _gr_system->_new_triangle->_v;
	_ipc->Step(h);
	Vector9d v = _gr_system->_new_triangle->_v;
	Vector9d a = (v - v_before) / h;

	if (v.lpNorm<1>() < _stuck_tolerance_velocity &&
	    a.lpNorm<1>() < _stuck_tolerance_acceleration) {
		_nearly_stop_successive_itrs++;
	}

	if (_nearly_stop_successive_itrs > 5) {
		for (const auto& constraint_pair : _ipc->_helper->_constraint_set) {
			
		}
	}
}

GeometryReconstructStepper::~GeometryReconstructStepper() {
	delete _ipc;
}