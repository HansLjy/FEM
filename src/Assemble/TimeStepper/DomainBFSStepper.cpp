//
// Created by hansljy on 11/29/22.
//

#include "DomainBFSStepper.h"
#include "Domain.h"

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
                new IPCBarrierTarget(GroupDomainIterator(_domains, _level_bar[i], _level_bar[i + 1]))
            );
        } else {
            _level_targets.push_back(
                new Target(GroupDomainIterator(_domains, _level_bar[i], _level_bar[i + 1]))
            );
        }
    }
}


void DomainBFSStepper::Step(double h) const {
    _domains[0]->BottomUpCalculation();
    for (int i = 0; i < _level; i++) {
        // [_level_bar[i], _level_bar[i + 1])
        for (int j = _level_bar[i]; j < _level_bar[i + 1]; j++) {
            _domains[j]->TopDownCalculationPrev();
            _domains[j]->SetObjectInfo();
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

void GroupDomainIterator::Forward() {
    _cur_itr->Forward();
    if (_cur_itr->IsDone()) {
        if (++_cur_domain == _end) {
            _is_done = true;
            return;
        }
        _cur_itr = std::shared_ptr<ObjectIterator>(new SystemIterator(*_domains[_cur_domain]));
    }
}

Object *GroupDomainIterator::GetObject() {
    return _cur_itr->GetObject();
}

