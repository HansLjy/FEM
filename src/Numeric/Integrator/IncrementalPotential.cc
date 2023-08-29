//
// Created by hansljy on 11/17/22.
//

#include "IncrementalPotential.h"
#include "Optimizer/Optimizer.h"
#include "Eigen/SparseQR"

namespace {
	const bool ip_registered = Factory<Integrator>::GetInstance()->Register("incremental-potential",
	[](const json& config) {
		return new IPIntegrator(config);
	});
}

IPIntegrator::IPIntegrator(const nlohmann::json &config) : Integrator(config) {
    const auto& optimizer_config = config["optimizer"];
    _optimizer = Factory<Optimizer>::GetInstance()->GetProduct(optimizer_config["type"], optimizer_config);
}

IPIntegrator::~IPIntegrator() {
    delete _optimizer;
}

void IPIntegrator::Step(Target &target, double h) const {
    const int dof = target.GetDOF();
    VectorXd x(dof), v(dof), force(dof);
    target.GetCoordinate(x);
    target.GetVelocity(v);
    target.GetExternalForce(force);
    SparseMatrixXd mass;
    target.GetMass(mass);

	VectorXd x_hat(x.size());

	if (_damping_enabled) {
		SparseMatrixXd energy_hessian;
		target.GetPotentialEnergyHessian(x, energy_hessian);
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

    auto func = [&x_hat, &mass, &h, &target] (const VectorXd& x) -> double {
        return 0.5 * (x - x_hat).transpose() * mass * (x - x_hat) + h * h * target.GetPotentialEnergy(x);
    };

    auto grad = [&x_hat, &mass, &h, &target, &dof] (const VectorXd& x, VectorXd& gradient) -> void {
        if (gradient.size() != dof) {
            gradient.resize(dof);
        }
        target.GetPotentialEnergyGradient(x, gradient);
        gradient = h * h * gradient + mass * (x - x_hat);
    };

    auto hes = [&mass, &h, &target] (const VectorXd& x, SparseMatrixXd& hessian) -> void {
        target.GetPotentialEnergyHessian(x, hessian);
        hessian = h * h * hessian + mass;
    };


    VectorXd x_next = x;
    _optimizer->Optimize(func, grad, hes, x_next);

	// VectorXd tmp_gradient;
	// grad(x_next, tmp_gradient);
	// spdlog::info("residue = {}", tmp_gradient.norm());

    VectorXd v_next = (x_next - x) / h;

    target.SetCoordinate(x_next);
    target.SetVelocity(v_next);
}