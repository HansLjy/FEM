#pragma once
#include "Coordinate.hpp"

class GridCoordinate : public BasicCoordinate {
public:
	template<class Data> static void SetCoordinate(Data* obj, const Ref<const VectorXd>& x) {
		obj->_x.head(obj->_dof) = x;
		obj->UpdateProxyPosition();
	}
};