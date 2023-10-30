//
// Created by hansljy on 11/23/22.
//

#ifndef FEM_CUBICSOLVER_H
#define FEM_CUBICSOLVER_H

#include "Pattern.h"

class CubicSolver {
public:
	static CubicSolver* GetProductFromConfig(const json& config);
    CubicSolver(double tolerance) : _tolerance(tolerance) {}
    virtual ~CubicSolver() = default;

    //<- return minimum root of Ax3 + Bx2 + Cx + D, in the interval [l, r]
	//<- if no root, return a value < l
    virtual double Solve(double A, double B, double C, double D, double l, double r) = 0;

protected:
    double _tolerance;
};


#endif //FEM_CUBICSOLVER_H
