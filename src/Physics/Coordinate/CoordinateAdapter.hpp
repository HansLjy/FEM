#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"

template<class Coordinate, class Derived>
class CoordinateAdapter {
public:
	int GetDOF() const {
		return Coordinate::GetDOF(static_cast<const Derived*>(this));
	}

	void GetCoordinate(Ref<VectorXd> x) const {
		Coordinate::GetCoordinate(static_cast<const Derived*>(this), x);
	}

	void GetVelocity(Ref<VectorXd> v) const {
		Coordinate::GetVelocity(static_cast<const Derived*>(this), v);
	}

	void SetCoordinate(const Ref<const VectorXd>& x) {
		Coordinate::SetCoordinate(static_cast<Derived*>(this), x);
	}

	void SetVelocity(const Ref<const VectorXd>& v) {
		Coordinate::SetVelocity(static_cast<Derived*>(this), v);
	}
};