#pragma once
#include "CubicSolver.h"

class CemCubicSolver : public CubicSolver {
public:
	explicit CemCubicSolver(const json& config) : CemCubicSolver(double(config["tolerance"])) {}
	CemCubicSolver(double tolerance) : CubicSolver(tolerance) {}
	double Solve(double A, double B, double C, double D, double l, double r) override;
};