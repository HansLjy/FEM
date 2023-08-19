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

IPIntegrator::IPIntegrator(const nlohmann::json &config) {
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

	// std::cerr << "Mass:\n" << mass.toDense() << std::endl;
	// std::cerr << "Force:\n" << force.transpose() << std::endl;

	// exit(-1);
	Eigen::SparseQR<SparseMatrixXd, Eigen::COLAMDOrdering<int>> SparseQR_solver;
	SparseQR_solver.compute(mass);
    // Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(mass);

    // Eigen::SelfAdjointEigenSolver<SparseMatrixXd> eigen_solver(mass);
    // std::cerr << eigen_solver.eigenvalues().transpose() << std::endl;

    // if (LDLT_solver.info() == Eigen::NumericalIssue) {
    //     std::cerr << "Mass matrix not SPD" << std::endl;
    //     exit(-1);
    // }

    // VectorXd a = LDLT_solver.solve(force);
	VectorXd a = SparseQR_solver.solve(force);
	// std::cerr << "a: " << a.transpose() << std::endl;

    VectorXd x_hat = x + h * v + h * h * a;
	// std::cerr << "x-hat: " << x_hat.transpose() << std::endl;

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

	// std::cerr << "x-next: " << x_next.transpose() << std::endl;
	// exit(-1);

    VectorXd v_next = (x_next - x) / h;

    target.SetCoordinate(x_next);
    target.SetVelocity(v_next);
}