//
// Created by hansljy on 11/29/22.
//

#include "DomainBFSStepper.h"
#include "Domain.h"

DEFINE_CLONE(Target, GroupDomainTarget)

DomainBFSStepper::DomainBFSStepper(const nlohmann::json &config) : TimeStepper(config) {
    const auto& integrator_config = config["integrator"];
    _integrator = IntegratorFactory::GetIntegrator(integrator_config["type"], integrator_config);
}

#include "Target/IPCBarrierTarget.h"
#include "Collision/Culling/HashCulling.h"

void DomainBFSStepper::Bind(System &system) {
    TimeStepper::Bind(system);
    auto& domain = dynamic_cast<Domain&>(system);

    _domains.clear();
    _level_bar.clear();
    _level = 0;

    int first = 0, last = 1, cur_level_end = 1;
    _level_bar.push_back(0);
    _domains.push_back(&domain);
    while (first < last) {
        auto& cur_domain = _domains[first++];
        for (auto& subdomain : cur_domain->_subdomains) {
            _domains.push_back(subdomain);
            last++;
        }
        if (first == cur_level_end) {
            _level_bar.push_back(cur_level_end);
            cur_level_end = last;
            _level++;
        }
    }

    _level_targets.clear();
    for (int i = 0; i < _level; i++) {
        if (_collision_aware) {
            _level_targets.push_back(
                CollisionAwareTargetFactory::GetCollisionAwareTarget(
                    GroupDomainTarget(_domains, _level_bar[i], _level_bar[i + 1]),
                    GroupDomainIterator(_domains, _level_bar[i], _level_bar[i + 1]),
                    _collision_config
               )
            );
        } else {
            _level_targets.push_back(new GroupDomainTarget(_domains, _level_bar[i], _level_bar[i + 1]));
        }
    }
}


void DomainBFSStepper::Step(double h) const {
    _domains[0]->BottomUpCalculation();
    for (int i = 0; i < _level; i++) {
        // [_level_bar[i], _level_bar[i + 1])
        for (int j = _level_bar[i]; j < _level_bar[i + 1]; j++) {
            _domains[j]->TopDownCalculationPrev();
            _domains[j]->SetObjectFrame();
        }
        const auto& target = _level_targets[i];
        VectorXd v(target->GetDOF()), v_new(target->GetDOF());
        target->GetVelocity(v);
        _integrator->Step(*target, h);
        target->GetVelocity(v_new);
        VectorXd a = (v_new - v) / h;

        int cur_row = 0;
        for (int j = _level_bar[i]; j < _level_bar[i + 1]; j++) {
            if (!_domains[j]->_subdomains.empty()) {
                _domains[j]->CalculateSubdomainFrame(a.segment(cur_row, _domains[j]->GetDOF()));
            }
            cur_row += _domains[j]->GetDOF();
        }
    }
}

DomainBFSStepper::~DomainBFSStepper() noexcept {
    delete _integrator;
    for (const auto& target : _level_targets) {
        delete target;
    }
}

GroupDomainTarget::GroupDomainTarget(const std::vector<Domain *> &domains, int begin, int end)
    : _domains(domains), _begin(begin), _end(end) {
    _dof = 0;
    for (int i = _begin; i < _end; i++) {
        _dof += _domains[i]->GetDOF();
        _targets.push_back(new DomainTarget(*_domains[i]));
    }
}

int GroupDomainTarget::GetDOF() const {
    return _dof;
}

#define ASSEMBLE_1D(FuncName, var)                                              \
    int cur_row = 0;                                                            \
    for (int i = _begin, j = 0; i < _end; i++, j++) {                           \
        _targets[j]->Get##FuncName(var.segment(cur_row, _targets[j]->GetDOF()));\
        cur_row += _domains[i]->GetDOF();                                       \
    }


void GroupDomainTarget::GetCoordinate(Ref<Eigen::VectorXd> x) const {
    ASSEMBLE_1D(Coordinate, x)
}

void GroupDomainTarget::GetVelocity(Ref<Eigen::VectorXd> v) const {
    ASSEMBLE_1D(Velocity, v)
}

#define DISPERSE_1D(FuncName, var)                                                  \
    int cur_row = 0;                                                                \
    for (int i = _begin, j = 0; i < _end; i++, j++) {                               \
        _targets[j]->Set##FuncName(var.segment(cur_row, _targets[j]->GetDOF()));    \
        cur_row += _domains[i]->GetDOF();                                           \
    }

void GroupDomainTarget::SetCoordinate(const Ref<const Eigen::VectorXd> &x) {
    DISPERSE_1D(Coordinate, x)
}

void GroupDomainTarget::SetVelocity(const Ref<const Eigen::VectorXd> &v) {
    DISPERSE_1D(Velocity, v)
}

#define ASSEMBLE_2D(FuncName)                                                           \
    int current_row = 0;                                                                \
    for (int i = _begin, j = 0; i < _end; i++, j++) {                                   \
        _targets[j]->Get##FuncName(coo, offset_x + current_row, offset_y + current_row);\
        current_row += _targets[j]->GetDOF();                                           \
    }

void GroupDomainTarget::GetMass(COO &coo, int offset_x, int offset_y) const {
    ASSEMBLE_2D(Mass)
}

double GroupDomainTarget::GetPotentialEnergy() const {
    double energy = 0;
    for (int i = _begin, j = 0; i < _end; i++, j++) {
        energy += _targets[j]->GetPotentialEnergy();
    }
    return energy;
}

double GroupDomainTarget::GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const {
    double energy = 0;
    int current_row = 0;
    for (int i = _begin, j = 0; i < _end; i++, j++) {
        energy += _targets[j]->GetPotentialEnergy(x.segment(current_row, _targets[j]->GetDOF()));
        current_row += _targets[j]->GetDOF();
    }
    return energy;
}

void GroupDomainTarget::GetPotentialEnergyGradient(Ref<Eigen::VectorXd> gradient) const {
    int current_row = 0;
    for (int i = _begin, j = 0; i < _end; i++, j++) {
        _targets[j]->GetPotentialEnergyGradient(gradient.segment(current_row, _targets[j]->GetDOF()));
        current_row += _targets[j]->GetDOF();
    }
}

void GroupDomainTarget::GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x,
                                                   Ref<Eigen::VectorXd> gradient) const {
    int current_row = 0;
    for (int i = _begin, j = 0; i < _end; i++, j++) {
        _targets[j]->GetPotentialEnergyGradient(x.segment(current_row, _targets[j]->GetDOF()), gradient.segment(current_row, _targets[j]->GetDOF()));
        current_row += _targets[j]->GetDOF();
    }
}

void GroupDomainTarget::GetPotentialEnergyHessian(COO &coo, int offset_x, int offset_y) const {
    int current_row = 0;
    for (int i = _begin, j = 0; i < _end; i++, j++) {
        _targets[j]->GetPotentialEnergyHessian(coo, offset_x + current_row, offset_y + current_row);
        current_row += _targets[j]->GetDOF();
    }
}

void GroupDomainTarget::GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x,
                                                  int offset_y) const {
    int current_row = 0;
    for (int i = _begin, j = 0; i < _end; i++, j++) {
        _targets[j]->GetPotentialEnergyHessian(x.segment(current_row, _targets[j]->GetDOF()), coo, offset_x + current_row, offset_y + current_row);
        current_row += _targets[j]->GetDOF();
    }
}

void GroupDomainTarget::GetExternalForce(Ref<Eigen::VectorXd> force) const {
    int current_row = 0;
    for (int i = _begin, j = 0; i < _end; i++, j++) {
        _targets[j]->GetExternalForce(force.segment(current_row, _targets[j]->GetDOF()));
        current_row += _targets[j]->GetDOF();
    }
}

VectorXd GroupDomainTarget::GetConstraint(const Eigen::VectorXd &x) const {
    // TODO
}

void GroupDomainTarget::GetConstraintGradient(SparseMatrixXd &gradient, const Eigen::VectorXd &x) const {
    // TODO
}

GroupDomainTarget::~GroupDomainTarget() {
    for (auto& target : _targets) {
        delete target;
    }
}

GroupDomainTarget::GroupDomainTarget(const GroupDomainTarget &rhs)
    : _domains(rhs._domains), _begin(rhs._begin), _end(rhs._end), _dof(rhs._dof) {
    for (auto& target : rhs._targets) {
        _targets.push_back(target->Clone());
    }
}

void GroupDomainIterator::Forward() {
    if (_cur_itr->IsDone()) {
        if (++_cur_domain == _end) {
            _is_done = true;
            return;
        }
        _cur_itr = _domains[_cur_domain]->GetIterator();
    } else {
        _cur_itr->Forward();
    }
}

Object *GroupDomainIterator::GetObject() {
    return _cur_itr->GetObject();
}

