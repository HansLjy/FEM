//
// Created by hansljy on 10/29/22.
//

#ifndef FEM_DERIVATIVETEST_H
#define FEM_DERIVATIVETEST_H

#include "FiniteDifference.h"

#define GenerateDerivative(data, physics, gradient_step, hessian_step) \
	const auto& energy_func = [&data, &physics] (const VectorXd& x) {					\
		return physics.GetPotential(data, x);											\
	};																					\
																						\
	const int dof = data->_dof;															\
	VectorXd x = VectorXd::Random(dof);													\
																						\
	const auto& numeric_gradient = FiniteDifferential(energy_func, x, gradient_step);	\
	const auto& analytic_gradient = physics.GetPotentialGradient(data, x);				\
																						\
	const auto& numeric_hessian = FiniteDifferential2(energy_func, x, hessian_step);	\
	COO coo;																			\
	physics.GetPotentialHessian(data, x, coo, 0, 0);									\
	SparseMatrixXd analytic_hessian(dof, dof);											\
	analytic_hessian.setFromTriplets(coo.begin(), coo.end());

#define PrintGradient() \
    std::cerr << "Numeric Gradient: \n" << numeric_gradient.transpose() << std::endl;               \
    std::cerr << "Analytic Gradient: \n" << analytic_gradient.transpose() << std::endl;             \
    std::cerr << "Gradient Difference: \n"															\
			  << (numeric_gradient - analytic_gradient).transpose() << std::endl;					\

#define PrintHessian() \
    std::cerr << "Numeric Hessian: \n" << numeric_hessian << std::endl;                         \
    std::cerr << "Analytic Hessian: \n" << analytic_hessian.toDense() << std::endl;             \
    std::cerr << "Hessian Difference: \n"														\
			  << (numeric_hessian - analytic_hessian).transpose() << std::endl;					\


#endif //FEM_DERIVATIVETEST_H
