//
// Created by hansljy on 10/8/22.
//

#include "System.h"
#include "Constraint/Constraint.h"
#include "spdlog/spdlog.h"

System::System(const nlohmann::json &config) : _DOF(0), _constraint_size(0) {
    const auto& objects_config = config["objects"];
    for (const auto& object_config : objects_config) {
        const auto& object = ObjectFactory::GetObject(object_config["type"], object_config);
        AddObject(*object, object_config["name"]);
        delete object;
    }
    const auto& constraints_config = config["constraints"];
    for (const auto& constraint_config : constraints_config) {
        const auto& constraint = ConstraintFactory::GetConstraint(*this, constraint_config);
        AddConstraint(*constraint);
        delete constraint;
    }

    const auto& external_forces_config = config["external-forces"];
    for (const auto& external_force_config : external_forces_config) {
        int idx = GetIndex(external_force_config["object-name"]);
        const auto& external_force = ExternalForceFactory::GetExternalForce(external_force_config["type"], external_force_config);
        GetObject(idx)->AddExternalForce(*external_force);
        delete external_force;
    }

    _target = new SystemTarget(*this);
}

int System::AddObject(const Object &obj, const std::string &name) {
    int new_idx = _objs.size();
    auto result = _index.insert(std::pair<std::string, int>(name, new_idx));
    if (!result.second) {
        spdlog::error("Name {} already exist in the system!", name);
        return -1;
    }
    _objs.push_back(obj.Clone());
    _DOF += obj.GetDOF();
    _constraint_size += obj.GetConstraintSize();
    return new_idx;
}

Object * System::GetObject(int idx) {
    return _objs[idx];
}

const Object *System::GetObject(int idx) const {
    return _objs[idx];
}

int System::GetOffset(int idx) const {
    return _offset[idx];
}

int System::AddConstraint(const Constraint &constraint) {
    _constraints.push_back(constraint.Clone());
    _constraint_size += constraint.GetSize();
    return _constraints.size() - 1;
}

int System::GetIndex(const std::string &name) const {
    const auto& result = _index.find(name);
    if (result == _index.end()) {
        spdlog::error("Name {} does not correspond to any objects in the system", name);
        return -1;
    }
    return (*result).second;
}


void SystemTarget::GetConstraintGradient(SparseMatrixXd &gradient, const Eigen::VectorXd &x) const {
    COO coo;
    int num_constraint = _system->_constraints.size();
    int current_constraint_offset = 0;
    for (int i = 0; i < num_constraint; i++) {
        _system->_constraints[i]->GetGradient(x, coo, current_constraint_offset);
        current_constraint_offset += _system->_constraints[i]->GetSize();
    }
    int current_object_offset = 0;
    for (const auto& obj : _system->_objs) {
        obj->GetInnerConstraintGradient(x.segment(current_object_offset, obj->GetDOF()), coo, current_constraint_offset, current_object_offset);
        current_constraint_offset += obj->GetConstraintSize();
        current_object_offset += obj->GetDOF();
    }
    gradient.resize(current_constraint_offset, x.size());
    gradient.setFromTriplets(coo.begin(), coo.end());
}

void System::UpdateSettings(const json &config) {
    int cur_offset = 0;
    const int size = _objs.size();
    if (_offset.size() != size) {
        _offset.resize(size);
    }
    for (int i = 0; i < size; i++) {
        _offset[i] = cur_offset;
        cur_offset += _objs[i]->GetDOF();
    }
    for (auto& constraint : _constraints) {
        for (int i = 0; i < constraint->GetObjectsNum(); i++) {
            constraint->SetOffset(i, _offset[constraint->GetObjectIndex(i)]);
        }
    }
}

std::unique_ptr<ObjectIterator> System::GetIterator() {
    return std::unique_ptr<ObjectIterator>(new SystemIterator(*this));
}

Target *System::GetTarget() {
    return _target;
}

System::~System(){
    for (const auto& obj : _objs) {
        delete obj;
    }
    delete _target;
}

#define Assemble1D(Value)                                                               \
    VectorXd var(_system->_DOF);                                                        \
    int current_row = 0;                                                                \
    for (const auto& obj : _system->_objs) {                                            \
        var.segment(current_row, obj->GetDOF()) = obj->Get##Value();                    \
        current_row += obj->GetDOF();                                                   \
    }                                                                                   \
    return var;

#define Assemble1DWithInfo(Value, rotation, position)                                   \
    VectorXd var(_system->_DOF);                                                        \
    int current_row = 0;                                                                \
    for (const auto& obj : _system->_objs) {                                            \
        var.segment(current_row, obj->GetDOF()) = obj->Get##Value(rotation, position);  \
        current_row += obj->GetDOF();                                                   \
    }                                                                                   \
    return var;

#define Assemble1DElseWhere(Value)\
    VectorXd var(_system->_DOF);                                                        \
    int current_row = 0;                                                                \
    for (const auto& obj : _system->_objs) {                                            \
        var.segment(current_row, obj->GetDOF())                                         \
            = obj->Get##Value(x.segment(current_row, obj->GetDOF()));                   \
        current_row += obj->GetDOF();                                                   \
    }                                                                                   \
    return var;

#define Assemble2D(var, Value)                                                      \
    int current_row = 0;                                                            \
    COO coo;                                                                        \
    for (const auto& obj : _system->_objs) {                                        \
        obj->Get##Value(coo, current_row, current_row);                             \
        current_row += obj->GetDOF();                                               \
    }                                                                               \
    var.resize(current_row, current_row);                                           \
    var.setFromTriplets(coo.begin(), coo.end());

#define Assemble2DElseWhere(var, Value)                                             \
    int current_row = 0;                                                            \
    COO coo;                                                                        \
    for (const auto& obj : _system->_objs) {                                        \
        obj->Get##Value(x.segment(current_row, obj->GetDOF()), coo, current_row, current_row);                             \
        current_row += obj->GetDOF();                                               \
    }                                                                               \
    var.resize(current_row, current_row);                                           \
    var.setFromTriplets(coo.begin(), coo.end());

VectorXd SystemTarget::GetCoordinate() const {
    Assemble1D(Coordinate)
}

VectorXd SystemTarget::GetVelocity() const {
    Assemble1D(Velocity)
}

#define Dessemble1D(Funcname, var)                                              \
    int current_row = 0;                                                        \
    for (const auto& obj : _system->_objs) {                                    \
        obj->Set##Funcname(var.segment(current_row, obj->GetDOF()));            \
        current_row += obj->GetDOF();                                           \
    }

void SystemTarget::SetCoordinate(const Eigen::VectorXd &x) {
    Dessemble1D(Coordinate, x)
}

void SystemTarget::SetVelocity(const Eigen::VectorXd &v) {
    Dessemble1D(Velocity, v)
}

void SystemTarget::GetMass(SparseMatrixXd &mass) const {
    Assemble2D(mass, Mass)
}

double SystemTarget::GetPotentialEnergy() const {
    double energy = 0;
    for (const auto & obj : _system->_objs) {
        energy += obj->GetPotential();
    }
    return energy;
}

double SystemTarget::GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const {
    double energy = 0;
    int current_row = 0;
    for (const auto& obj : _system->_objs) {
        energy += obj->GetPotential(x.segment(current_row, obj->GetDOF()));
        current_row += obj->GetDOF();
    }
    return energy;
}

VectorXd SystemTarget::GetPotentialEnergyGradient() const {
    Assemble1D(PotentialGradient)
}

VectorXd SystemTarget::GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x) const {
    Assemble1DElseWhere(PotentialGradient)
}

void SystemTarget::GetPotentialEnergyHessian(SparseMatrixXd &hessian) const {
    Assemble2D(hessian, PotentialHessian)
}

void SystemTarget::GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, SparseMatrixXd &hessian) const {
    Assemble2DElseWhere(hessian, PotentialHessian)
}

VectorXd SystemTarget::GetExternalForce() const {
    Assemble1DWithInfo(ExternalForce, Matrix3d::Identity(), Vector3d::Zero())
}


VectorXd SystemTarget::GetConstraint(const VectorXd &x) const {
    VectorXd constraint(_system->_constraint_size);
    const int num_constraint = _system->_constraints.size();
    int current_constraint_offset = 0;
    for (int i = 0; i < num_constraint; i++) {
        constraint.block(current_constraint_offset, 0, _system->_constraints[i]->GetSize(), 1)
            = _system->_constraints[i]->GetValue(x);
        current_constraint_offset += _system->_constraints[i]->GetSize();
    }
    int current_object_offset = 0;
    for (const auto& obj : _system->_objs) {
        constraint.block(current_constraint_offset, 0, obj->GetConstraintSize(), 1)
            = obj->GetInnerConstraint(x.segment(current_object_offset, obj->GetDOF()));
        current_constraint_offset += obj->GetConstraintSize();
        current_object_offset += obj->GetDOF();
    }
    return constraint;
}

SystemIterator::SystemIterator(System &system)
        : ObjectIterator(system._objs.size() == 0), _system(&system), _obj_id(0), _cur_size(system._objs.size()) {}

void SystemIterator::Forward() {
    _obj_id++;
    if (_obj_id == _cur_size) {
        _is_done = true;
    }
}

Object *SystemIterator::GetObject() {
    return _system->_objs[_obj_id];
}

Matrix3d SystemIterator::GetRotation() {
    return Matrix3d::Identity();
}

Vector3d SystemIterator::GetTranslation() {
    return Vector3d::Zero();
}

#include "Tree/TreeDomain.h"

BEGIN_DEFINE_XXX_FACTORY(System)
    ADD_PRODUCT("system", System)
    ADD_PRODUCT("tree-domain", TreeDomain)
END_DEFINE_XXX_FACTORY