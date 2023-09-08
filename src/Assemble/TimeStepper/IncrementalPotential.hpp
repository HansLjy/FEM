#pragma once

#include "Optimizer/Optimizer.h"
#include "Assembler/Assembler.hpp"
#include "TimeStepper.hpp"

template<class System>
class IncrementalPotentialTimeStepper : public TimeStepper<System> {
public:
	explicit IncrementalPotentialTimeStepper(const json& config);

	void Step(double h) override;

	~IncrementalPotentialTimeStepper();

private:
	bool _damping_enabled;
	double _rayleigh_coef_mass = 0;
	double _rayleigh_coef_stiffness = 0;
	Optimizer* _optimizer;
	Assembler<System>* _assembler;
};

template<class System>
IncrementalPotentialTimeStepper<System>::IncrementalPotentialTimeStepper(const json& config)
: _optimizer(Factory<Optimizer>::GetInstance()->GetProduct(config["optimizer"]["type"], config["optimizer"])),
  _assembler(Factory<Assembler<System>>::GetInstance()->GetProduct(config["assembler"]["type"], config["assembler"])),
  _damping_enabled(config["damping-enabled"]) {
	if (_damping_enabled) {
		_rayleigh_coef_mass = config["rayleigh-coef-mass"];
		_rayleigh_coef_stiffness = config["rayleigh-coef-stiffness"];
	}
}

template<class System>
IncrementalPotentialTimeStepper<System>::~IncrementalPotentialTimeStepper() {
	delete _optimizer;
	delete _assembler;
}

template<class System>
void IncrementalPotentialTimeStepper<System>::Step(double h) {
	_assembler->BindSystem(this->_system);

    const int dof = _assembler->GetDOF();
    VectorXd x(dof), v(dof), force(dof);
    _assembler->GetCoordinate(x);
    _assembler->GetVelocity(v);
    _assembler->GetExternalForce(force);
    SparseMatrixXd mass;
    _assembler->GetMass(mass);

	VectorXd x_hat(x.size());

	if (_damping_enabled) {
		SparseMatrixXd energy_hessian;
		_assembler->GetPotentialEnergyHessian(x, energy_hessian);
		VectorXd Mv = mass * v;
		mass += h * (_rayleigh_coef_mass * mass + _rayleigh_coef_stiffness * energy_hessian);
		Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(mass);
		if (LDLT_solver.info() != Eigen::Success) {
			std::cerr << "System Matrix Not PSD" << std::endl;
			exit(-1);
		}
		x_hat = x + LDLT_solver.solve(h * h * force + h * Mv);
	} else {
		Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(mass);
		if (LDLT_solver.info() != Eigen::Success) {
			std::cerr << "System Matrix Not PSD" << std::endl;
			exit(-1);
		}
		x_hat = x + h * v + h * h * LDLT_solver.solve(force);
	}

    auto func = [&x_hat, &mass, &h, this] (const VectorXd& x) -> double {
        return 0.5 * (x - x_hat).transpose() * mass * (x - x_hat) + h * h * _assembler->GetPotentialEnergy(x);
    };

    auto grad = [&x_hat, &mass, &h, &dof, this] (const VectorXd& x, VectorXd& gradient) -> void {
        if (gradient.size() != dof) {
            gradient.resize(dof);
        }
        _assembler->GetPotentialEnergyGradient(x, gradient);
        gradient = h * h * gradient + mass * (x - x_hat);
    };

    auto hes = [&mass, &h, this] (const VectorXd& x, SparseMatrixXd& hessian) -> void {
        _assembler->GetPotentialEnergyHessian(x, hessian);
        hessian = h * h * hessian + mass;
    };


    VectorXd x_next = x;
    _optimizer->Optimize(func, grad, hes, x_next);

	// VectorXd tmp_gradient;
	// grad(x_next, tmp_gradient);
	// spdlog::info("residue = {}", tmp_gradient.norm());

    VectorXd v_next = (x_next - x) / h;

    _assembler->SetCoordinate(x_next);
    _assembler->SetVelocity(v_next);

}