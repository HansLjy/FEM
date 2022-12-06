//
// Created by hansljy on 11/17/22.
//

#include "IncrementalPotential.h"
#include "Optimizer/Optimizer.h"

IPIntegrator::IPIntegrator(const nlohmann::json &config) {
    const auto& optimizer_config = config["optimizer"];
    _optimizer = OptimizerFactory::GetOptimizer(optimizer_config["type"], optimizer_config);
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

    Eigen::SimplicialLDLT LDLT_solver(mass);
    VectorXd a = LDLT_solver.solve(force);

    VectorXd x_hat = x + h * v + h * h * a;

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

    VectorXd v_next = (x_next - x) / h;

    target.SetCoordinate(x_next);
    target.SetVelocity(v_next);
}