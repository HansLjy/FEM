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

    int time_stamp = 0;

    int step = 0;
    SparseMatrixXd hessian;
    VectorXd gradient(dof), x_prev(x);
    while(step++ < _max_iter) {
        ipc_target.ComputeConstraintSet(x, time_stamp++);
        target.GetPotentialEnergyHessian(x, hessian);
        target.GetPotentialEnergyGradient(x, gradient);
        double prev_energy = h * h * target.GetPotentialEnergy(x) + 0.5 * (x - x_hat).transpose() * mass * (x - x_hat);

        LDLT_solver.compute(mass + h * h * hessian);
        VectorXd p = - LDLT_solver.solve(mass * (x - x_hat) + h * h * gradient);

        if (p.norm() / p.size() < _tolerance * h) {
            break;
        }

        /* contact aware line search */
        double alpha = ipc_target.GetMaxStep(p);
        VectorXd x_next;
        while (true) {
            x_next = x + alpha * p;
            ipc_target.ComputeConstraintSet(x_next, time_stamp++);
            if (h * h * target.GetPotentialEnergy(x_next) + 0.5 * (x_next - x_hat).transpose() * mass * (x_next - x_hat) < prev_energy) {
                break;
            }
            alpha *= 0.5;
        }
        x = x_next;
    }
    if (step >= _max_iter) {
        spdlog::warn("Barrier-aware Newton not converge");
    } else {
        spdlog::info("Barrier-aware Newton converge in {} steps", step);
    }

    target.SetCoordinate(x);
    target.SetVelocity((x - x_prev) / h);
}