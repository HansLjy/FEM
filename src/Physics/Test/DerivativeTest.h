//
// Created by hansljy on 10/29/22.
//

#ifndef FEM_DERIVATIVETEST_H
#define FEM_DERIVATIVETEST_H

#include "FiniteDifference.h"

#define GetEnergyFunction(func, ValueName, obj)  \
    auto func = [&obj] (const VectorXd& x) {        \
        return obj.Get##ValueName(x);               \
    };

#define GenerateDerivatives(obj, ValueName, Randomize, step_gradient, step_hessian) \
    GetEnergyFunction(func, ValueName, obj)                                         \
                                                                                    \
    Randomize(obj);                                                                 \
    VectorXd x(obj.GetDOF());                                                       \
	obj.GetCoordinate(x);															\
    auto numeric_gradient = FiniteDifferential(func, x, step_gradient);             \
    auto analytic_gradient = obj.Get##ValueName##Gradient(x);                       \
                                                                                    \
    auto numeric_hessian = FiniteDifferential2(func, x, step_hessian);              \
    COO coo;                                                                        \
    obj.Get##ValueName##Hessian(x, coo, 0, 0);                                      \
    SparseMatrixXd analytic_hessian(obj.GetDOF(), obj.GetDOF());                    \
    analytic_hessian.setFromTriplets(coo.begin(), coo.end());


#define PrintGradient() \
    std::cerr << "Numeric Gradient: \n" << numeric_gradient.transpose() << std::endl;               \
    std::cerr << "Analytic Gradient: \n" << analytic_gradient.transpose() << std::endl;             \
    std::cerr << "Gradient Difference: \n" << numeric_gradient - analytic_gradient << std::endl;    \

#define PrintHessian() \
    std::cerr << "Numeric Hessian: \n" << numeric_hessian << std::endl;                         \
    std::cerr << "Analytic Hessian: \n" << analytic_hessian.toDense() << std::endl;             \
    std::cerr << "Hessian Difference: \n" << numeric_hessian - analytic_hessian << std::endl;   \


#endif //FEM_DERIVATIVETEST_H
