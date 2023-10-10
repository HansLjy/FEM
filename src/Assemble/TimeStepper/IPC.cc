
#include "IPC.hpp"
#include "Pattern.h"

namespace {
	const bool ipc_registered = Factory<TimeStepper>::GetInstance()->Register("ipc", [](const json& config){
		return new IPC(config);
	});
}

IPC::IPC(const json& config) :
    _max_iter(config["max-iteration"]),
    _tolerance(config["tolerance"]),
    _ipc_energy(config["d-hat"], config["kappa"]),
    _max_step_estimator(config["d-hat"]),
    _constraint_set_generator(config["d-hat"], config["grid-length"], config["hash-table-size"]) {

}

void IPC::BindObjects(
    const typename std::vector<Object>::const_iterator &begin,
    const typename std::vector<Object>::const_iterator &end
) {
    _coord_assembler.BindObjects(begin, end);
    _mass_assembler.BindObjects(begin, end);
    _energy_assembler.BindObjects(begin, end);
    _external_force_assembler.BindObjects(begin, end);

    _ipc_energy.BindObjects(begin, end);
    _max_step_estimator.BindObjects(begin, end);
    _constraint_set_generator.BindObjects(begin, end);
}

void IPC::Step(double h) {
	const int dof = _coord_assembler.GetDOF();

    VectorXd x(dof), v(dof), force(dof);
    _coord_assembler.GetCoordinate(x);
    _coord_assembler.GetVelocity(v);
    _external_force_assembler.GetExternalForce(force);

    SparseMatrixXd mass;
    _mass_assembler.GetMass(mass);

    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(mass);
    VectorXd a = LDLT_solver.solve(force);

    VectorXd x_hat = x + h * v + h * h * a;

	double residue;
    int step = 0;
    SparseMatrixXd hessian;
    VectorXd gradient(dof), x_prev(x);
	static int fuck_itr = 0;
    fuck_itr++;

    std::vector<PrimitivePair> constraint_set;
    while(step++ < _max_iter) {
        _constraint_set_generator.ComputeConstraintSet(x, constraint_set);

		COO coo;
		hessian.resize(dof, dof);
        _energy_assembler.GetPotentialHessian(x, coo, 0, 0);
		_ipc_energy.GetBarrierEnergyHessian(constraint_set, coo, 0, 0);
		hessian.setFromTriplets(coo.begin(), coo.end());

        _energy_assembler.GetPotentialGradient(x, gradient);
		gradient += _ipc_energy.GetBarrierEnergyGradient(constraint_set);
        double prev_energy =
            h * h * (_energy_assembler.GetPotentialEnergy(x) + _ipc_energy.GetBarrierEnergy(constraint_set))
            + 0.5 * (x - x_hat).transpose() * mass * (x - x_hat);

        LDLT_solver.compute(mass + h * h * hessian);
        VectorXd p = - LDLT_solver.solve(mass * (x - x_hat) + h * h * gradient);

        if ((residue = p.lpNorm<Eigen::Infinity>()) < _tolerance * h) {
            break;
        }

        /* contact aware line search */
        double alpha = _max_step_estimator.GetMaxStep(p);
        // spdlog::info("Max step: {}", alpha);
        VectorXd x_next;
        while (true) {
            x_next = x + alpha * p;
            _constraint_set_generator.ComputeConstraintSet(x_next, constraint_set);
            auto current_energy = h * h * (_energy_assembler.GetPotentialEnergy(x_next) + _ipc_energy.GetBarrierEnergy(constraint_set)) + 0.5 * (x_next - x_hat).transpose() * mass * (x_next - x_hat);
            // spdlog::info("Current energy: {}, Previous energy: {}", current_energy, prev_energy);
            if (current_energy <= prev_energy) {
                break;
            }
            alpha *= 0.5;
        }
        // spdlog::info("alpha = {}", alpha);
        x = x_next;
    }
    // spdlog::info("Itr: {}", fuck_itr);
    if (step >= _max_iter) {
        spdlog::warn("Barrier-aware Newton not converge, residue = {}", residue);
        // exit(-1);
    } else {
        // spdlog::info("Barrier-aware Newton converge in {} steps", step);
    }

	// std::cerr << x.transpose() << std::endl;
    _coord_assembler.SetCoordinate(x);
    _coord_assembler.SetVelocity((x - x_prev) / h);
}