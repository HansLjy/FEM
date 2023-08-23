//
// Created by hansljy on 10/8/22.
//

#ifndef FEM_INTEGRATOR_H
#define FEM_INTEGRATOR_H

#include "Pattern.h"
#include "Target.h"
#include "spdlog/spdlog.h"

class Integrator {
public:
	explicit Integrator(const json& config) : Integrator(config["damping-enabled"], config["rayleigh-coef-mass"], config["rayleigh-coef-stiffness"]) {}
	Integrator(bool damping_enabled, double rayleigh_coef_mass, double rayleigh_coef_stiffness)
		: _damping_enabled(damping_enabled), _rayleigh_coef_mass(rayleigh_coef_mass), _rayleigh_coef_stiffness(rayleigh_coef_stiffness) {}
    virtual void Step(Target &target, double h) const = 0;
    virtual ~Integrator() = default;

protected:
	bool _damping_enabled;
	double _rayleigh_coef_mass, _rayleigh_coef_stiffness;
};

#endif //FEM_INTEGRATOR_H
