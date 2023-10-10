#include "Assembler.hpp"
#include "Pattern.h"

template<>
Caster<Energied>* Caster<Energied>::_the_factory = nullptr;

template<>
Caster<Massed>* Caster<Massed>::_the_factory = nullptr;

template<>
Caster<Coordinated>* Caster<Coordinated>::_the_factory = nullptr;

template<>
Caster<ExternalForced>* Caster<ExternalForced>::_the_factory = nullptr;

void CoordinateAssembler::BindObjects(
    const typename std::vector<Object>::const_iterator &begin,
    const typename std::vector<Object>::const_iterator &end
) {
    InterfaceContainer::BindObjects(begin, end);
    _total_dof = 0;
    for (const auto& obj : _objs) {
        _total_dof += obj.GetDOF();
    }
}

int CoordinateAssembler::GetDOF() const {
    return _total_dof;
}

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
    int current_row = 0;
    for (const auto& obj : _objs) {
        obj.GetMass(coo, current_row + offset_x, current_row + offset_y);
        current_row += obj.GetDOF();
    } 
}

void EnergyAssembler::BindObjects(
    const typename std::vector<Object>::const_iterator &begin,
    const typename std::vector<Object>::const_iterator &end
) {
    InterfaceContainer::BindObjects(begin, end);
    _total_dof = 0;
    for (const auto& obj : _objs) {
        _total_dof += obj.GetDOF();
    }
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

void EnergyAssembler::GetPotentialHessian(const Ref<const VectorXd>& x, SparseMatrixXd& hessian) const {
	COO coo;
	GetPotentialHessian(x, coo, 0, 0);
	hessian.resize(_total_dof, _total_dof);
	hessian.setFromTriplets(coo.begin(), coo.end());
}

void EnergyAssembler::GetPotentialGradient(const Ref<const Eigen::VectorXd> &x, Ref<Eigen::VectorXd> gradient) const {
    int cur_offset = 0;
    for (const auto& obj : _objs) {
        gradient.segment(cur_offset, obj.GetDOF()) = obj.GetPotentialGradient(x.segment(cur_offset, obj.GetDOF()));
        cur_offset += obj.GetDOF();
    }
}

void EnergyAssembler::GetPotentialHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const {
    int current_row = 0;
    for (const auto& obj : _objs) {
        obj.GetPotentialHessian(
            x.segment(current_row, obj.GetDOF()),
            coo,
            current_row + offset_x,
            current_row + offset_y
        );
        current_row += obj.GetDOF();
    }
}

void ExternalForceAssembler::GetExternalForce(Ref<Eigen::VectorXd> force) const {
    int cur_offset = 0;
    for (const auto& obj : this->_objs) {
        force.segment(cur_offset, obj.GetDOF()) = obj.GetExternalForce();
        cur_offset += obj.GetDOF();
    }
}