#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"

class BasicCoordinate {
public:
	template<class Data> void Initialize(Data* data) {}
	template<class Data> static int GetDOF(const Data* obj);
	template<class Data> static void GetCoordinate(const Data* obj, Ref<VectorXd> x);
	template<class Data> static void GetVelocity(const Data* obj, Ref<VectorXd> v);
	template<class Data> static void SetCoordinate(Data* obj, const Ref<const VectorXd>& x);
	template<class Data> static void SetVelocity(Data* obj, const Ref<const VectorXd>& v);
};

template<class ProxyCoordinate>
class ReducedCoordinate : public BasicCoordinate {
public:
	template<class Data> static void SetCoordinate(Data* obj, const Ref<const VectorXd>& x);
	template<class Data> static void SetVelocity(Data* obj, const Ref<const VectorXd>& v);
};

template<class Data> int BasicCoordinate::GetDOF(const Data *obj) {
	return obj->_dof;
}

template<class Data> void BasicCoordinate::GetCoordinate(const Data* obj, Ref<VectorXd> x) {
	x = obj->_x.head(obj->_dof);
}

template<class Data> void BasicCoordinate::GetVelocity(const Data* obj, Ref<VectorXd> v) {
	v = obj->_v.head(obj->_dof);
}

template<class Data> void BasicCoordinate::SetCoordinate(Data* obj, const Ref<const VectorXd>& x) {
	obj->_x.head(obj->_dof) = x;
}

template<class Data> void BasicCoordinate::SetVelocity(Data *obj, const Ref<const VectorXd> &v) {
	obj->_v.head(obj->_dof) = v;
}

template<class ProxyCoordinate>
template<class Data> void ReducedCoordinate<ProxyCoordinate>::SetCoordinate(Data* obj, const Ref<const VectorXd>& x) {
	BasicCoordinate::SetCoordinate(obj, x);
	ProxyCoordinate::SetCoordinate(obj->_proxy, obj->_base * x + obj->_shift);
}

template<class ProxyCoordinate>
template<class Data> void ReducedCoordinate<ProxyCoordinate>::SetVelocity(Data* obj, const Ref<const VectorXd>& v) {
	BasicCoordinate::SetVelocity(obj, v);
	ProxyCoordinate::SetVelocity(obj->_proxy, obj->_base * v);
}
