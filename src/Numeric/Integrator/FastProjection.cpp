//
// Created by hansljy on 10/8/22.
//

#include "FastProjection.h"
#include "Eigen/SparseCholesky"
#include "spdlog/spdlog.h"
#include "Timer.h"

FastProjectionIntegrator::FastProjectionIntegrator(double tolerance, int max_step) : _tolerance(tolerance), _max_step(max_step) {}
FastProjectionIntegrator::FastProjectionIntegrator(const nlohmann::json &config) : FastProjectionIntegrator(config["tolerance"], config["max_step"]) {}

void FastProjectionIntegrator::Step(Target &target, double h) const {
////    START_TIMING(stepping)
//
    // step without constraint
    SparseMatrixXd mass, hessian;
    target.GetMass(mass);
//    START_TIMING(hessian_calculation)
    target.GetPotentialEnergyHessian(hessian);
//    STOP_TIMING("hessian calculation", hessian_calculation)
    SparseMatrixXd A = mass + h * h * hessian;
//    SparseMatrixXd A = mass;

    const VectorXd& x = target.GetCoordinate();
    const VectorXd& v = target.GetVelocity();

    VectorXd energy_gradient = target.GetPotentialEnergyGradient();
    VectorXd external_force = target.GetExternalForce();
//    std::cout << energy_gradient.transpose() << std::endl;

    VectorXd b = mass * v + h * external_force - h * energy_gradient;
//    std::cout << b.transpose() << std::endl;

    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT;
    LDLT.compute(A);
    if (LDLT.info() == Eigen::NumericalIssue) {
        spdlog::warn("A is not SPD!");
    }
    VectorXd v_next = LDLT.solve(b);
//    std::cout << "v_next: \n" << v_next.transpose() << std::endl;

    // Project back to constraint manifold
    VectorXd x_next = x + h * v_next;
    VectorXd C = target.GetConstraint(x_next);

    Eigen::LDLT<MatrixXd> LDLT_dense;
    int step = 0;
    while(std::abs(C.norm() / C.size()) > _tolerance) {
//        std::cout << "x_next = " << x_next.transpose() << std::endl;
//        std::cout << "constraint = " << C.transpose() << std::endl;

        SparseMatrixXd nabla_c;
        target.GetConstraintGradient(nabla_c, x_next);

        Matrix nabla_c_dense = nabla_c.toDense();

        LDLT.compute(mass);
        MatrixXd A_tmp = LDLT.solve(nabla_c_dense.transpose());
        MatrixXd A_final = nabla_c * A_tmp;

        LDLT_dense.compute(A_final);
        VectorXd lambda = LDLT_dense.solve(C) / (h * h);
        x_next = x_next - h * h * A_tmp * lambda;
        if (++step > _max_step) {
            break;
        }
        C = target.GetConstraint(x_next);
    }
    if (step > _max_step) {
//        spdlog::warn("Fast projection fails to converge, error of last step: {}", C.norm() / C.size());
    } else {
//        spdlog::info("Fast projection converges in {} steps", step);
    }

    v_next = (x_next - x) / h;
//    std::cout << "Coordinate: " << x_next.transpose() << std::endl;

    target.SetVelocity(v_next);
    target.SetCoordinate(x_next);
//    STOP_TIMING("stepping", stepping)
}