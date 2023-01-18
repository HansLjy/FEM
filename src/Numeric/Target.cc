//
// Created by hansljy on 12/6/22.
//

#include "Target.h"
#include "Object.h"

Target::Target(const std::vector<Object*> & objs, int begin, int end, const json& config) {
    _dof = 0;
    for (int i = begin; i < end; i++) {
        _objs.push_back(objs[i]);
        _offsets.push_back(_dof);
        _dof += objs[i]->GetDOF();
    }
}

#define ASSEMBLE_1D(FuncName, var)  \
    int cur_offset = 0;             \
    for (const auto& obj : _objs) { \
        obj->Get##FuncName(var.segment(cur_offset, obj->GetDOF())); \
        cur_offset += obj->GetDOF();                                   \
    }

void Target::GetCoordinate(Ref<Eigen::VectorXd> x) const {
    ASSEMBLE_1D(Coordinate, x)
}

void Target::GetVelocity(Ref<Eigen::VectorXd> v) const {
    ASSEMBLE_1D(Velocity, v)
}

#define DISPERSE_1D(FuncName, var) \
    int cur_offset = 0;             \
    for (const auto& obj : _objs) { \
        obj->Set##FuncName(var.segment(cur_offset, obj->GetDOF())); \
        cur_offset += obj->GetDOF();                                \
    }

void Target::SetCoordinate(const Ref<const Eigen::VectorXd> &x) {
    DISPERSE_1D(Coordinate, x)
}

void Target::SetVelocity(const Ref<const Eigen::VectorXd> &v) {
    DISPERSE_1D(Velocity, v)
}

#define ASSEMBLE_2D(FuncName) \
    int current_row = 0;                                                            \
    for (const auto& obj : _objs) {                                                 \
        obj->Get##FuncName(coo, current_row + offset_x, current_row + offset_y);    \
        current_row += obj->GetDOF();                                               \
    }                                                                               \


void Target::GetMass(COO &coo, int offset_x, int offset_y) const {
    ASSEMBLE_2D(Mass)
}

double Target::GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const {
    double energy = 0;
    int cur_offset = 0;
    for (const auto& obj : _objs) {
        energy += obj->GetPotential(x.segment(cur_offset, obj->GetDOF()));
        cur_offset += obj->GetDOF();
    }
    return energy;
}

#define ASSEMBLE_1D_ELSEWHERE(FuncName, X, var) \
    int cur_offset = 0;             \
    for (const auto& obj : _objs) { \
        var.segment(cur_offset, obj->GetDOF()) = obj->Get##FuncName(X.segment(cur_offset, obj->GetDOF())); \
        cur_offset += obj->GetDOF();                                   \
    }


void Target::GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x, Ref<Eigen::VectorXd> gradient) const {
    ASSEMBLE_1D_ELSEWHERE(PotentialGradient, x, gradient)
}

#define ASSEMBLE_2D_ELSEWHERE(FuncName, X) \
    int current_row = 0;                                                            \
    for (const auto& obj : _objs) {                                                 \
        obj->Get##FuncName(X.segment(current_row, obj->GetDOF()), coo, current_row + offset_x, current_row + offset_y);    \
        current_row += obj->GetDOF();                                               \
    }

void
Target::GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const {
    ASSEMBLE_2D_ELSEWHERE(PotentialHessian, x)
}

void Target::GetExternalForce(Ref<Eigen::VectorXd> force) const {
    int cur_offset = 0;
    for (const auto& obj : _objs) {
        force.segment(cur_offset, obj->GetDOF()) = obj->GetExternalForce();
        cur_offset += obj->GetDOF();
    }
}

#include "Target/IPCBarrierTarget.h"

Target* TargetFactory::GetTarget(const std::vector<Object *> objs, int begin, int end, const std::string& type, const json &config) {
    if (type == "basic") {
        return new Target(objs, begin, end, config);
    }
    if (type == "collision-barriered") {
        return new IPCBarrierTarget(objs, begin, end, config);
    }
    return nullptr;
}