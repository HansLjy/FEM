#include "Assembler.hpp"
#include "Pattern.h"


#define ASSEMBLE_1D(FuncName, var)										\
    int cur_offset = 0;													\
    for (const auto& obj : (this->_objs)) {								\
        obj.Get##FuncName(var.segment(cur_offset, obj.GetDOF()));		\
        cur_offset += obj.GetDOF();									    \
    }

void CoordinateAssembler::GetCoordinate(Ref<Eigen::VectorXd> x) const {
    ASSEMBLE_1D(Coordinate, x)
}

void CoordinateAssembler::GetVelocity(Ref<Eigen::VectorXd> v) const {
    ASSEMBLE_1D(Velocity, v)
}

#undef ASSEMBLE_1D

#define DISPERSE_1D(FuncName, var)									\
    int cur_offset = 0;												\
    for (auto& obj : this->_objs) {							        \
        obj.Set##FuncName(var.segment(cur_offset, obj.GetDOF()));   \
        cur_offset += obj.GetDOF();                                 \
    }

void CoordinateAssembler::SetCoordinate(const Ref<const Eigen::VectorXd> &x) const {
    DISPERSE_1D(Coordinate, x)
}

void CoordinateAssembler::SetVelocity(const Ref<const Eigen::VectorXd> &v) const {
    DISPERSE_1D(Velocity, v)
}

#undef DISPERSE_1D

#define ASSEMBLE_2D(FuncName)														\
    int current_row = 0;                                                            \
    for (const auto& obj : this->_objs) {											\
        obj.Get##FuncName(coo, current_row + offset_x, current_row + offset_y);     \
        current_row += obj.GetDOF();                                                \
    }                                                                               \


void MassAssembler::BindObjects(
	const typename std::vector<Object>::const_iterator& begin,
	const typename std::vector<Object>::const_iterator& end
) {
	InterfaceContainer<Massed>::BindObjects(begin, end);
	_total_dof = 0;
	for (const auto& obj : _objs) {
		_total_dof += obj.GetDOF();
	}
}

void MassAssembler::GetMass(SparseMatrixXd& mass) const {
	COO coo;
	GetMass(coo, 0, 0);
	mass.resize(_total_dof, _total_dof);
	mass.setFromTriplets(coo.begin(), coo.end());
}

void MassAssembler::GetMass(COO &coo, int offset_x, int offset_y) const {
    ASSEMBLE_2D(Mass)
}

double EnergyAssembler::GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const {
    double energy = 0;
    int cur_offset = 0;
    for (const auto& obj : this->_objs) {
        energy += obj.GetPotential(x.segment(cur_offset, obj.GetDOF()));
        cur_offset += obj.GetDOF();
    }
    return energy;
}

#undef ASSEMBLE_2D

#define ASSEMBLE_1D_ELSEWHERE(FuncName, X, var) \
    int cur_offset = 0;             \
    for (const auto& obj : this->_objs) { \
        var.segment(cur_offset, obj->GetDOF()) = obj->Get##FuncName(X.segment(cur_offset, obj->GetDOF())); \
        cur_offset += obj->GetDOF();                                   \
    }

void GeneralAssembler::GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x, Ref<Eigen::VectorXd> gradient) const {
    ASSEMBLE_1D_ELSEWHERE(PotentialGradient, x, gradient)
}

#undef ASSEMBLE_1D_ELSEWHERE

#define ASSEMBLE_2D_ELSEWHERE(FuncName, X) \
    int current_row = 0;                                                            \
    for (const auto& obj : this->_objs) {                                                 \
        obj->Get##FuncName(X.segment(current_row, obj->GetDOF()), coo, current_row + offset_x, current_row + offset_y);    \
        current_row += obj->GetDOF();                                               \
    }

void GeneralAssembler::GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const {
    ASSEMBLE_2D_ELSEWHERE(PotentialHessian, x)
}

void GeneralAssembler::GetExternalForce(Ref<Eigen::VectorXd> force) const {
    int cur_offset = 0;
    for (const auto& obj : this->_objs) {
        force.segment(cur_offset, obj->GetDOF()) = obj->GetExternalForce();
        cur_offset += obj->GetDOF();
    }
}

#undef ASSEMBLE_2D_ELSEWHERE

template<>
Factory<Assembler>* Factory<Assembler>::_the_factory = nullptr;



void Assembler::GetPotentialEnergyHessian(const Ref<const VectorXd>& x, SparseMatrixXd& hessian) const {
	COO coo;
	GetPotentialEnergyHessian(x, coo, 0, 0);
	hessian.resize(GetDOF(), GetDOF());
	hessian.setFromTriplets(coo.begin(), coo.end());
}