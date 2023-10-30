#include "CemCubicSolver.hpp"
#include "cyPolynomial.h"

CemCubicSolver* CemCubicSolver::CreateFromConfig(const json& config) {
	return new CemCubicSolver(config["tolerance"]);
}

double CemCubicSolver::Solve(double A, double B, double C, double D, double l, double r) {
	double root;
	double coefs[] = {D, C, B, A};
	bool success = cy::PolynomialFirstRoot<3>(root, coefs, l, r, _tolerance);
	if (!success) {
		return l - 1;
	} else {
		return root;
	}
}