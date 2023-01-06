//
// Created by hansljy on 11/23/22.
//

#ifndef FEM_BISECTIONCUBICSOLVER_H
#define FEM_BISECTIONCUBICSOLVER_H

#include "CubicSolver.h"
#include <functional>

class BisectionCubicSolver : public CubicSolver {
public:
    explicit BisectionCubicSolver(const json& config) : BisectionCubicSolver(double(config["tolerance"])) {}
    BisectionCubicSolver(double tolerance) : CubicSolver(tolerance) {}
    double Solve(double A, double B, double C, double D, double l, double r) override;

protected:
    double FindRoot(const std::function<double(double)>& f, double l, double r);
};

#endif //FEM_BISECTIONCUBICSOLVER_H
