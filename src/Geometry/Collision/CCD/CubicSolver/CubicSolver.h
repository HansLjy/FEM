//
// Created by hansljy on 11/23/22.
//

#ifndef FEM_CUBICSOLVER_H
#define FEM_CUBICSOLVER_H

#include "Pattern.h"

class CubicSolver {
public:
    CubicSolver(double tolerance) : _tolerance(tolerance) {}

    //<- return minimum root in the interval
    //<- a > 0 must hold
    virtual double Solve(double A, double B, double C, double D, double l, double r) = 0;

protected:
    double _tolerance;
};

DECLARE_XXX_FACTORY(CubicSolver)

#endif //FEM_CUBICSOLVER_H
