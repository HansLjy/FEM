//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_FINITEDIFFERENCE_H
#define FEM_FINITEDIFFERENCE_H

#include "../EigenAll.h"

VectorXd FiniteDifferential(const std::function<double(const VectorXd&)>& func, VectorXd x);
MatrixXd FiniteDifferential2(const std::function<double(const VectorXd&)>& func, VectorXd x);

#endif //FEM_FINITEDIFFERENCE_H
