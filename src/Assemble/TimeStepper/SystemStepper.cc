//
// Created by hansljy on 11/28/22.
//

#include "SystemStepper.h"
#include "Target.h"

namespace {
	const bool system_stepper_registered = Factory<TimeStepper>::GetInstance()->Register("system-stepper",
	[](const json& config) {
		return new SystemStepper(config);
	});
}

SystemStepper::SystemStepper(const json &config) : TimeStepper(config) {
    const auto& integrator_config = config["integrator"];
    _integrator = Factory<Integrator>::GetInstance()->GetProduct(integrator_config["type"], integrator_config);
}

void SystemStepper::Bind(System &system) {
    TimeStepper::Bind(system);
    _target = TargetFactory::GetTarget(system._objs, 0, system._objs.size(), _target_config["type"], _target_config);
}

void SystemStepper::Step(double h) const {
    _integrator->Step(*_target, h);
}

SystemStepper::~SystemStepper() {
    delete _integrator;
}