#pragma once
#include "EigenAll.h"
#include "JsonUtil.h"

template<template<class> class EnergyModel, class Derived>
class EnergyModelImplementation {
public:
	double GetPotential(const Ref<const VectorXd> &x) const {
		return _model.GetPotential(static_cast<Derived*>(this), x);
	}

	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const {
		return _model.GetPotentialGradient(static_cast<Derived*>(this), x);
	}

	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
		_model.GetPotentialHessian(static_cast<Derived*>(this), x, coo, x_offset, y_offset);
	}

private:
	EnergyModel<Derived> _model;
};