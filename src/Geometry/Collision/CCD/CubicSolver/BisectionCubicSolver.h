//
// Created by hansljy on 11/23/22.
//

#ifndef FEM_BISECTIONCUBICSOLVER_H
#define FEM_BISECTIONCUBICSOLVER_H

#include "CubicSolver.h"

class BisectionCubicSolver : public CubicSolver {
public:
    double Solve(double A, double B, double C, double D, double l, double r) override;
};

#endif //FEM_BISECTIONCUBICSOLVER_H
