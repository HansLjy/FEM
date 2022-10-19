//
// Created by hansljy on 10/8/22.
//

#include "System.h"
#include "Constraint/Constraint.h"
#include "spdlog/spdlog.h"

System::System(int size)
    : _DOF(0), _size(size), _constraint_size(0),
      _objs(std::vector<Object*>(size)),
      _used(std::vector<bool>(size)),
      _next_free(std::vector<int>(size)),
      _index(), _first_free(0) {
    for (int i = 0; i < size; i++) {
        _next_free[i] = i + 1;
        _used[i] = false;
        _objs[i] = nullptr;
    }
}

int System::AddObject(const Object &obj, const std::string &name) {
    auto result = _index.insert(std::pair<std::string, int>(name, _first_free));
    if (!result.second) {
        spdlog::error("Name {} already exist in the system!", name);
        return -1;
    }
    _used[_first_free] = true;
    _objs[_first_free] = obj.Clone();
    auto idx = _first_free;
    _first_free = _next_free[_first_free];

    if (_first_free >= _size) {
        // Need to reallocate
        _objs.resize(_size * 2);
        _used.resize(_size * 2);
        for (int i = _size; i < _size * 2; i++) {
            _used[i] = false;
        }
        _next_free.resize(_size * 2);
        for (int i = _size; i < _size * 2; i++) {
            _next_free[i] = i + 1;
        }
        _size *= 2;
    }
    _DOF += obj.GetDOF();
    _constraint_size += obj.GetConstraintSize();
    return idx;
}

bool System::DeleteObject(int idx) {
    if (idx < 0 || idx >= _size || !_used[idx]) {
        spdlog::error("Index {} does not correspond to any objects in the system", idx);
        return false;
    }

    _used[idx] = false;
    _next_free[idx] = _first_free;
    _first_free = idx;
    _DOF -= _objs[idx]->GetDOF();
    _constraint_size -= _objs[idx]->GetConstraintSize();
    delete _objs[idx];

    for (auto itr = _index.begin(); itr != _index.end(); itr++) {
        if ((*itr).second == idx) {
            _index.erase(itr);
            break;
        }
    }
    return true;
}

bool System::DeleteObject(const std::string &name) {
    const auto& result = _index.find(name);
    if (result == _index.end()) {
        spdlog::error("Name {} does not correspond to any objects in the system", name);
        return false;
    }
    int idx = (*result).second;

    _used[idx] = false;
    _next_free[idx] = _first_free;
    _first_free = idx;
    _DOF -= _objs[idx]->GetDOF();
    _constraint_size -= _objs[idx]->GetConstraintSize();
    delete _objs[idx];

    _index.erase(result);
    return true;
}

Object * System::GetObject(int idx) {
    return _objs[idx];
}

const Object *System::GetObject(int idx) const {
    return _objs[idx];
}


#define Assemble1D(Value)                                                       \
    VectorXd var(_DOF);                                                         \
    int current_row = 0;                                                        \
    for (int i = 0; i < _size; i++) {                                           \
        if (_used[i]) {                                                         \
            const auto& obj = _objs[i];                                         \
            var.block(current_row, 0, obj->GetDOF(), 1) = obj->Get##Value();    \
            current_row += obj->GetDOF();                                       \
        }                                                                       \
    }                                                                           \
    return var;

#define Assemble2D(var, Value)                                  \
    int current_row = 0;                                        \
    COO coo;                                                    \
    for (int i = 0; i < _size; i++) {                           \
        if (_used[i]) {                                         \
            const auto& obj = _objs[i];                         \
            obj->Get##Value(coo, current_row, current_row);     \
            current_row += obj->GetDOF();                       \
        }                                                       \
    }                                                           \
    var.resize(current_row, current_row);                       \
    var.setFromTriplets(coo.begin(), coo.end());                \

VectorXd System::GetCoordinate() const {
    Assemble1D(Coordinate)
}

VectorXd System::GetVelocity() const {
    Assemble1D(Velocity)
}

#define Dessemble1D(Funcname, var)                                              \
    int current_row = 0;                                                        \
    for (int i = 0; i < _size; i++) {                                           \
        if (_used[i]) {                                                         \
            const auto& obj = _objs[i];                                         \
            obj->Set##Funcname(var.block(current_row, 0, obj->GetDOF(), 1));    \
            current_row += obj->GetDOF();                                       \
        }                                                                       \
    }

void System::SetCoordinate(const Eigen::VectorXd &x) {
    Dessemble1D(Coordinate, x)
}

void System::SetVelocity(const Eigen::VectorXd &v) {
    Dessemble1D(Velocity, v)
}

void System::GetMass(SparseMatrixXd &mass) const {
    Assemble2D(mass, Mass)
}

double System::GetEnergy() const {
    double energy = 0;
    for (int i = 0; i < _size; i++) {
        if (_used[i]) {
            energy += _objs[i]->GetEnergy();
        }
    }
    return energy;
}

VectorXd System::GetEnergyGradient() const {
    Assemble1D(EnergyGradient)
}

void System::GetEnergyHessian(SparseMatrixXd &hessian) const {
    Assemble2D(hessian, EnergyHessian)
}

int System::GetIndex(const std::string &name) const {
    const auto& result = _index.find(name);
    if (result == _index.end()) {
        spdlog::error("Name {} does not correspond to any objects in the system", name);
        return -1;
    }

    return (*result).second;
}

int System::AddConstraint(const Constraint &constraint) {
    _constraints.push_back(constraint.Clone());
    _constraint_size += constraint.GetSize();
    return _constraints.size() - 1;
}

bool System::DeleteConstraint(int idx) {
    if (idx < 0 || idx >= _constraints.size()) {
        spdlog::error("Index {} does not correspond to any constraint in the system", idx);
        return false;
    }
    delete _constraints[idx];
    _constraint_size -= _constraints[idx]->GetSize();
    _constraints.erase(_constraints.begin() + idx);
    return true;
}

VectorXd System::GetConstraint(const VectorXd &x) const {
    VectorXd constraint(_constraint_size);
    const int num_constraint = _constraints.size();
    int current_constraint_offset = 0;
    for (int i = 0; i < num_constraint; i++) {
        constraint.block(current_constraint_offset, 0, _constraints[i]->GetSize(), 1)
            = _constraints[i]->GetValue(x);
        current_constraint_offset += _constraints[i]->GetSize();
    }
    int current_object_offset = 0;
    for (int i = 0; i < _size; i++) {
        if (_used[i]) {
            constraint.block(current_constraint_offset, 0, _objs[i]->GetConstraintSize(), 1)
                = _objs[i]->GetInnerConstraint(x.segment(current_object_offset, _objs[i]->GetDOF()));
            current_constraint_offset += _objs[i]->GetConstraintSize();
            current_object_offset += _objs[i]->GetDOF();
        }
    }
    return constraint;
}

void System::GetConstraintGradient(SparseMatrixXd &gradient, const Eigen::VectorXd &x) const {
    COO coo;
    int num_constraint = _constraints.size();
    int current_constraint_offset = 0;
    for (int i = 0; i < num_constraint; i++) {
        _constraints[i]->GetGradient(x, coo, current_constraint_offset);
        current_constraint_offset += _constraints[i]->GetSize();
    }
    int current_object_offset = 0;
    for (int i = 0; i < _size; i++) {
        if (_used[i]) {
            _objs[i]->GetInnerConstraintGradient(x.segment(current_object_offset, _objs[i]->GetDOF()), coo, current_constraint_offset, current_object_offset);
            current_constraint_offset += _objs[i]->GetConstraintSize();
            current_object_offset += _objs[i]->GetDOF();
        }
    }
    gradient.resize(current_constraint_offset, x.size());
    gradient.setFromTriplets(coo.begin(), coo.end());
}

void System::UpdateSettings() {
    if (_offset.size() != _size) {
        _offset.resize(_size);
    }
    int cur_offset = 0;
    for (int i = 0; i < _size; i++) {
        if (_used[i]) {
            _offset[i] = cur_offset;
            cur_offset += _objs[i]->GetDOF();
        }
    }
    for (auto& constraint : _constraints) {
        for (int i = 0; i < constraint->GetObjectsNum(); i++) {
            constraint->SetOffset(i, _offset[constraint->GetObjectIndex(i)]);
        }
    }
}

