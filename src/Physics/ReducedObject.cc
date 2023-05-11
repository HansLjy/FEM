#include "ReducedObject.hpp"

void ReducedObject::GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	COO coo_full;
    _proxy->GetPotentialHessian(_base * x + _shift, coo_full, 0, 0);
    SparseMatrixXd proxy_hessian(_proxy->GetDOF(), _proxy->GetDOF());
    proxy_hessian.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd reduced_hessian = _base.transpose() * proxy_hessian * _base;
	SparseToCOO(reduced_hessian, coo, x_offset, y_offset);
}

void ReducedObject::GetMass(COO &coo, int x_offset, int y_offset) const {
	COO coo_full;
    _proxy->GetMass(coo_full, 0, 0);
    SparseMatrixXd mass(_proxy->GetDOF(), _proxy->GetDOF());
    mass.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd mass_reduced = _base.transpose() * mass * _base;
	SparseToCOO(mass_reduced, coo, x_offset, y_offset);
}