#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"

template <class ProxyEnergyModel>
class ReducedEnergyModel {
public:
	explicit ReducedEnergyModel(const json& config) : _proxy_model(config) {}
	template<class Derived> double GetPotential(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived> VectorXd GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived> void GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;

protected:
	ProxyEnergyModel _proxy_model;
};

template <class ProxyEnergyModel>
template <class Derived>
double ReducedEnergyModel<ProxyEnergyModel>::GetPotential(Derived* obj, const Ref<const VectorXd> &x) const {
	return _proxy_model.GetPotential(obj->_proxy, obj->_base * x + obj->_shift);
}

template <class ProxyEnergyModel>
template <class Derived>
VectorXd ReducedEnergyModel<ProxyEnergyModel>::GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const {
	return obj->_base.transpose() * _proxy_model.GetPotentialGradient(obj->_proxy, obj->_base * x + obj->_shift);
}

template <class ProxyEnergyModel>
template <class Derived>
void ReducedEnergyModel<ProxyEnergyModel>::GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	COO coo_full;
	_proxy_model.GetPotentialHessian(obj->_proxy, obj->_base * x + obj->_shift, coo_full, 0, 0);
    SparseMatrixXd proxy_hessian(obj->_proxy->_dof, obj->_proxy->_dof);
    proxy_hessian.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd reduced_hessian = obj->_base.transpose() * proxy_hessian * obj->_base;
	SparseToCOO(reduced_hessian, coo, x_offset, y_offset);
}
