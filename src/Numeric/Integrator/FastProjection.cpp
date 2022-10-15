//
// Created by hansljy on 10/8/22.
//

#include "FastProjection.h"
#include "Eigen/SparseCholesky"
#include "spdlog/spdlog.h"

FastProjectionIntegrator::FastProjectionIntegrator(double tolerance, int max_step) : _tolerance(tolerance), _max_step(max_step) {}
FastProjectionIntegrator::FastProjectionIntegrator(const nlohmann::json &config) : FastProjectionIntegrator(config["tolerance"], config["max_step"]) {}

void FastProjectionIntegrator::Step(Target &target, double h, VectorXd &x_next, VectorXd &v_next) const {
    // step without constraint
    SparseMatrixXd mass;
    target.GetMass(mass);
//    std::cout << mass.toDense() << std::endl;
    const VectorXd x = target.GetCoordinate();
    const VectorXd v = target.GetVelocity();

    /* Currently, we only use sympletic euler */

    VectorXd energy_gradient = target.GetEnergyGradient();
//    std::cout << energy_gradient.transpose() << std::endl;

    VectorXd b = mass * v - h * energy_gradient;
//    std::cout << b.transpose() << std::endl;

    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT;
    LDLT.compute(mass);
    v_next = LDLT.solve(b);
//    std::cout << "v_next: \n" << v_next.transpose() << std::endl;

    // Project back to constraint manifold
    x_next = x + h * v_next;
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
        spdlog::warn("Fast projection fails to converge, error of last step: {}", C.norm() / C.size());
    } else {
        spdlog::info("Fast projection converges in {} steps", step);
    }

    v_next = (x_next - x) / h;
//    std::cout << "Coordinate: " << x_next.transpose() << std::endl;
}