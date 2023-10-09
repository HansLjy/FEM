#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"

template<class Coordinate, class Derived>
class CoordinateImplementation {
public:
	int GetDOF() {
		return Coordinate::GetDOF(static_cast<Derived*>(this));
	}

	void GetCoordinate(Ref<VectorXd> x) {
		Coordinate::GetCoordinate(static_cast<Derived*>(this), x);
	}

	void GetVelocity(Ref<VectorXd> v) {
		Coordinate::GetVelocity(static_cast<Derived*>(this), v);
	}

	void SetCoordinate(const Ref<const VectorXd>& x) {
		Coordinate::SetCoordinate(static_cast<Derived*>(this), x);
	}

	void SetVelocity(const Ref<const VectorXd>& v) {
		Coordinate::SetVelocity(static_cast<Derived*>(this), v);
	}
};