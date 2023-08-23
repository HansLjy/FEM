#include "Physics.hpp"
#include "unsupported/Eigen/KroneckerProduct"

template<class ProxyModel>
class GridBasedPhysics : public BasicPhysics {
public:
	template<class Data> static void SetCoordinate(Data* data, const Ref<const VectorXd>& x);
	template<class Data> static void SetVelocity(Data* data, const Ref<const VectorXd>& v);

	template<class Data> static void GetMass(const Data* data, COO& coo, int x_offset, int y_offset);

	template<class Data> double GetPotential(const Data* data, const Ref<const VectorXd> &x) const;
	template<class Data> VectorXd GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const;
	template<class Data> void GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;

protected:
	ProxyModel _proxy_model;
};

template<class ProxyModel>
template<class Data> void GridBasedPhysics<ProxyModel>::SetCoordinate(Data* data, const Ref<const VectorXd>& x) {
	BasicPhysics::SetCoordinate(data, x);
	ProxyModel::SetCoordinate(data->_proxy, Eigen::kroneckerProduct(data->_base_compressed, Matrix3d::Identity()) * x);
}

template<class ProxyModel>
template<class Data> void GridBasedPhysics<ProxyModel>::SetVelocity(Data *data, const Ref<const VectorXd> &v) {
	BasicPhysics::SetVelocity(data, v);
	ProxyModel::SetVelocity(data->_proxy, Eigen::kroneckerProduct(data->_base_compressed, Matrix3d::Identity()) * v);
}

template<class ProxyModel>
template<class Data> void GridBasedPhysics<ProxyModel>::GetMass(const Data* data, COO& coo, int x_offset, int y_offset) {
	const int num_grids = data->_num_grids;
	for (int i = 0; i < num_grids; i++) {
		coo.push_back(Tripletd(x_offset + i, y_offset + i, data->_mass(i)));
	}
}

template<class ProxyModel>
template<class Data> double GridBasedPhysics<ProxyModel>::GetPotential(const Data* data, const Ref<const VectorXd> &x) const {
	return _proxy_model.GetPotential(data->_proxy, Eigen::kroneckerProduct(data->_base_compressed, Matrix3d::Identity()) * x);
}

template<class ProxyModel>
template<class Data> VectorXd GridBasedPhysics<ProxyModel>::GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const {
	return Eigen::kroneckerProduct(data->_base_compressed, Matrix3d::Identity()).transpose()
		   * _proxy_model.GetPotentialGradient(data->_proxy, Eigen::kroneckerProduct(data->_base_compressed, Matrix3d::Identity()) * x);
}

template<class ProxyModel>
template<class Data> void GridBasedPhysics<ProxyModel>::GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	COO coo_full;
	const auto& base = Eigen::kroneckerProduct(data->_base_compressed, Matrix3d::Identity()).transpose();
	_proxy_model.GetPotentialHessian(data->_proxy, base * x, coo_full, 0, 0);
    SparseMatrixXd proxy_hessian(ProxyModel::GetDOF(data->_proxy), ProxyModel::GetDOF(data->_proxy));
    proxy_hessian.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd reduced_hessian = base.transpose() * proxy_hessian * base;
	SparseToCOO(reduced_hessian, coo, x_offset, y_offset);
}