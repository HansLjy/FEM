#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"

template <class ProxyEnergyModel>
class ReducedEnergyModel {
public:
	explicit ReducedEnergyModel(const json& config) : _proxy_model(config) {}
	template<class Data> void Initialize(const Data* data) {_proxy_model.Initialize(data->_proxy);}
	template<class Data> double GetPotential(const Data* obj, const Ref<const VectorXd> &x) const;
	template<class Data> VectorXd GetPotentialGradient(const Data* obj, const Ref<const VectorXd> &x) const;
	template<class Data> void GetPotentialHessian(const Data* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;

protected:
	ProxyEnergyModel _proxy_model;
};

template <class EnergyModelA, class EnergyModelB>
class SumOfEnergyModel {
public:
	explicit SumOfEnergyModel(const json& config) : _model_a(config["model-1"]), _model_b("model-2") {}

	template<class Data> void Initialize(const Data* data) {
		_model_a.Initialize(data);
		_model_b.Initialize(data);
	}

	template<class Data> double GetPotential(const Data* data, const Ref<const VectorXd> &x) const {
		return _model_a.GetPotential(data, x) + _model_b.GetPotential(data, x);
	}

	template<class Data> VectorXd GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const {
		return _model_a.GetPotentialGradient(data, x) + _model_b.GetPotentialGradient(data, x);
	}

	template<class Data> void GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
		_model_a.GetPotentialHessian(data, x, coo, x_offset, y_offset);
		_model_b.GetPotentialHessian(data, x, coo, x_offset, y_offset);
	}

protected:
	EnergyModelA _model_a;
	EnergyModelB _model_b;
};

template<class ProxyEnergyModel>
template<class Data> double ReducedEnergyModel<ProxyEnergyModel>::GetPotential(const Data* obj, const Ref<const VectorXd> &x) const {
	return _proxy_model.GetPotential(obj->_proxy, obj->_base * x + obj->_shift);
}

template<class ReducedModel>
template<class Data> VectorXd ReducedEnergyModel<ReducedModel>::GetPotentialGradient(const Data* obj, const Ref<const VectorXd> &x) const {
	return obj->_base.transpose() * _proxy_model.GetPotentialGradient(obj->_proxy, obj->_base * x + obj->_shift);
}

template<class ReducedModel>
template<class Data> void ReducedEnergyModel<ReducedModel>::GetPotentialHessian(const Data* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	COO coo_full;
	_proxy_model.GetPotentialHessian(obj->_proxy, obj->_base * x + obj->_shift, coo_full, 0, 0);
    SparseMatrixXd proxy_hessian(obj->_proxy->_dof, obj->_proxy->_dof);
    proxy_hessian.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd reduced_hessian = obj->_base.transpose() * proxy_hessian * obj->_base;
	SparseToCOO(reduced_hessian, coo, x_offset, y_offset);
}
