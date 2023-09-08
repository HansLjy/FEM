#include "GeneralAssembler.hpp"

void GeneralAssembler::BindObjects(
	const typename std::vector<Object *>::const_iterator &begin,
	const typename std::vector<Object *>::const_iterator &end) {
	Assembler::BindObjects(begin, end);
    _dof = 0;
	for (auto itr = begin; itr != end; ++itr) {
        _offsets.push_back(_dof);
        _dof += (*itr)->GetDOF();
	}
}

void GeneralAssembler::BindObjects(const std::vector<Object*>& objs) {
	BindObjects(objs.begin(), objs.end());
}

#define ASSEMBLE_1D(FuncName, var)										\
    int cur_offset = 0;													\
    for (const auto& obj : (this->_objs)) {								\
        obj->Get##FuncName(var.segment(cur_offset, obj->GetDOF()));		\
        cur_offset += obj->GetDOF();									\
    }

void GeneralAssembler::GetCoordinate(Ref<Eigen::VectorXd> x) const {
    ASSEMBLE_1D(Coordinate, x)
}

void GeneralAssembler::GetVelocity(Ref<Eigen::VectorXd> v) const {
    ASSEMBLE_1D(Velocity, v)
}

#undef ASSEMBLE_1D

#define DISPERSE_1D(FuncName, var)									\
    int cur_offset = 0;												\
    for (const auto& obj : this->_objs) {							\
        obj->Set##FuncName(var.segment(cur_offset, obj->GetDOF())); \
        cur_offset += obj->GetDOF();                                \
    }

void GeneralAssembler::SetCoordinate(const Ref<const Eigen::VectorXd> &x) {
    DISPERSE_1D(Coordinate, x)
}

void GeneralAssembler::SetVelocity(const Ref<const Eigen::VectorXd> &v) {
    DISPERSE_1D(Velocity, v)
}

#undef DISPERSE_1D

#define ASSEMBLE_2D(FuncName)														\
    int current_row = 0;                                                            \
    for (const auto& obj : this->_objs) {											\
        obj->Get##FuncName(coo, current_row + offset_x, current_row + offset_y);    \
        current_row += obj->GetDOF();                                               \
    }                                                                               \

void GeneralAssembler::GetMass(COO &coo, int offset_x, int offset_y) const {
    ASSEMBLE_2D(Mass)
}

double GeneralAssembler::GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const {
    double energy = 0;
    int cur_offset = 0;
    for (const auto& obj : this->_objs) {
        energy += obj->GetPotential(x.segment(cur_offset, obj->GetDOF()));
        cur_offset += obj->GetDOF();
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