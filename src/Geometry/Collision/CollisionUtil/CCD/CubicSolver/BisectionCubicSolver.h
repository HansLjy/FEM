//
// Created by hansljy on 11/23/22.
//

#ifndef FEM_BISECTIONCUBICSOLVER_H
#define FEM_BISECTIONCUBICSOLVER_H

#include "CubicSolver.h"
#include <functional>

class BisectionCubicSolver : public CubicSolver {
public:
	static BisectionCubicSolver* CreateFromConfig(const json& config);
    ~BisectionCubicSolver() override = default;
    BisectionCubicSolver(double tolerance) : CubicSolver(tolerance) {}
    double Solve(double A, double B, double C, double D, double l, double r) override;

protected:
    double FindRoot(const std::function<double(double)>& f, double l, double r);
};

#endif //FEM_BISECTIONCUBICSOLVER_H
