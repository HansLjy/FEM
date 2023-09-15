#include "Assembler.hpp"
#include "Pattern.h"

template<>
Factory<Assembler>* Factory<Assembler>::_the_factory = nullptr;

void Assembler::GetMass(SparseMatrixXd& mass) const {
	COO coo;
	GetMass(coo, 0, 0);
	mass.resize(GetDOF(), GetDOF());
	mass.setFromTriplets(coo.begin(), coo.end());
}

void Assembler::GetPotentialEnergyHessian(const Ref<const VectorXd>& x, SparseMatrixXd& hessian) const {
	COO coo;
	GetPotentialEnergyHessian(x, coo, 0, 0);
	hessian.resize(GetDOF(), GetDOF());
	hessian.setFromTriplets(coo.begin(), coo.end());
}


void Assembler::BindObjects(const typename std::vector<Object *>::const_iterator &begin, const typename std::vector<Object *>::const_iterator &end) {
	for (auto itr = begin; itr != end; ++itr) {
		_objs.push_back((*itr));
	}
}

void Assembler::BindObjects(const std::vector<Object*>& objs) {
	BindObjects(objs.begin(), objs.end());
}

void Assembler::BindSystem(System &system) {
	BindObjects(system.GetObjs());
}
