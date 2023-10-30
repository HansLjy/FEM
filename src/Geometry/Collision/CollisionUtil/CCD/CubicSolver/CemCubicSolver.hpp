#pragma once
#include "CubicSolver.h"

class CemCubicSolver : public CubicSolver {
public:
	static CemCubicSolver* CreateFromConfig(const json& config);
	CemCubicSolver(double tolerance) : CubicSolver(tolerance) {}
	double Solve(double A, double B, double C, double D, double l, double r) override;
};