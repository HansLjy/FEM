//
// Created by hansljy on 10/13/22.
//

#include "FiniteDifference.h"

// Take the derivative of func at x
VectorXd FiniteDifferential(const std::function<double(const VectorXd&)>& func, VectorXd x, double step) {
    int size = x.size();
    VectorXd gradient(size);
    for (int i = 0; i < size; i++) {
        double tmp = x(i);
        x(i) = tmp + step;
        double f_plus = func(x);
        x(i) = tmp - step;
        double f_minus = func(x);
        x(i) = tmp;
        gradient(i) = (f_plus - f_minus) / (2 * step);
    }
    return gradient;
}

MatrixXd FiniteDifferential2(const std::function<double(const VectorXd&)>& func, VectorXd x, double step) {
    const double difference [] = {step, -step};
    int size = x.size();
    MatrixXd hessian(size, size);
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < i; j++) {
            double f[2][2];
            const double xi = x(i), xj = x(j);
            for (int ii : {0, 1}) {
                for (int jj : {0, 1}) {
                    x(i) = xi + difference[ii];
                    x(j) = xj + difference[jj];
                    f[ii][jj] = func(x);
                }
            }
            x(i) = xi;
            x(j) = xj;
            hessian(i, j) = hessian(j, i) = (f[0][0] + f[1][1] - f[0][1] - f[1][0]) / (4 * step * step);
        }
        double f = func(x);
        double f_difference[2];
        const double xi = x(i);
        for (int ii : {0, 1}) {
            x(i) = xi + difference[ii];
            f_difference[ii] = func(x);
        }
        hessian(i, i) = (f_difference[0] + f_difference[1] - 2 * f) / (step * step);
    }
    return hessian;
}