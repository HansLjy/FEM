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
    VectorXd x = target.GetCoordinate();
    VectorXd v = target.GetVelocity();
    VectorXd force = target.GetExternalForce();
    SparseMatrixXd mass;
    target.GetMass(mass);

    Eigen::SimplicialLDLT LDLT_solver(mass);
    VectorXd a = LDLT_solver.solve(force);

    VectorXd x_hat = x + h * v + h * h * a;

    auto func = [&x_hat, &mass, &h, &target] (const VectorXd& x) -> double {
        return 0.5 * (x - x_hat).transpose() * mass * (x - x_hat) + h * h * target.GetPotentialEnergy(x);
    };

    auto grad = [&x_hat, &mass, &h, &target] (const VectorXd& x) -> VectorXd {
        return mass * (x - x_hat) + h * h * target.GetPotentialEnergyGradient(x);
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