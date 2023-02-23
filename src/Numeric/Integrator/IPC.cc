//
// Created by hansljy on 12/5/22.
//

#include "IPC.h"
#include "EigenAll.h"
#include "Target/IPCBarrierTarget.h"

void IPC::Step(Target &target, double h) const {
    auto& ipc_target = dynamic_cast<IPCBarrierTarget&>(target);
    const int dof = target.GetDOF();
    VectorXd x(dof), v(dof), force(dof);
    target.GetCoordinate(x);
    target.GetVelocity(v);
    target.GetExternalForce(force);

    SparseMatrixXd mass;
    target.GetMass(mass);

    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(mass);
    VectorXd a = LDLT_solver.solve(force);

    VectorXd x_hat = x + h * v + h * h * a;

	double residue;
    int step = 0;
    SparseMatrixXd hessian;
    VectorXd gradient(dof), x_prev(x);
	static int fuck_itr = 0;
    fuck_itr++;
    while(step++ < _max_iter) {
        ipc_target.ComputeConstraintSet(x);
        target.GetPotentialEnergyHessian(x, hessian);
        target.GetPotentialEnergyGradient(x, gradient);
        double prev_energy = h * h * target.GetPotentialEnergy(x) + 0.5 * (x - x_hat).transpose() * mass * (x - x_hat);

        LDLT_solver.compute(mass + h * h * hessian);
        VectorXd p = - LDLT_solver.solve(mass * (x - x_hat) + h * h * gradient);

        if ((residue = p.lpNorm<Eigen::Infinity>()) < _tolerance * h) {
            break;
        }

        /* contact aware line search */
        double alpha = ipc_target.GetMaxStep(p);
        // spdlog::info("Max step: {}", alpha);
        VectorXd x_next;
        while (true) {
            x_next = x + alpha * p;
            ipc_target.ComputeConstraintSet(x_next);
            auto current_energy = h * h * target.GetPotentialEnergy(x_next) + 0.5 * (x_next - x_hat).transpose() * mass * (x_next - x_hat);
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
    target.SetCoordinate(x);
    target.SetVelocity((x - x_prev) / h);
}