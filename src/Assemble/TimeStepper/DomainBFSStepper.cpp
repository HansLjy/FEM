//
// Created by hansljy on 11/29/22.
//

#include "DomainBFSStepper.h"
#include "Domain.h"

DomainBFSStepper::DomainBFSStepper(const nlohmann::json &config) {
    const auto& integrator_config = config["integrator"];
    _integrator = IntegratorFactory::GetIntegrator(integrator_config["type"], integrator_config);
}

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
}

void DomainBFSStepper::Step(double h) const {
    _domains[0]->BottomUpCalculation();
    for (int i = 0; i < _level; i++) {
        // [_level_bar[i], _level_bar[i + 1])
        for (int j = _level_bar[i]; j < _level_bar[i + 1]; j++) {
            _domains[j]->TopDownCalculationPrev();
        }
        GroupDomainTarget target(_domains, _level_bar[i], _level_bar[i + 1]);
        VectorXd v(target.GetDOF()), v_new(target.GetDOF());
        target.GetVelocity(v);
        _integrator->Step(target, h);
        target.GetVelocity(v_new);
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
}

GroupDomainTarget::GroupDomainTarget(const std::vector<Domain *> &domains, int begin, int end)
    : _domains(domains), _begin(begin), _end(end) {
    _dof = 0;
    for (int i = _begin; i < _end; i++) {
        _dof += _domains[i]->GetDOF();
    }
}

int GroupDomainTarget::GetDOF() const {
    return _dof;
}

#define ASSEMBLE_1D(FuncName, var)                                  \
    int cur_row = 0;                                                \
    for (int i = _begin; i < _end; i++) {                           \
        DomainTarget target(*_domains[i]);                          \
        target.Get##FuncName(var.segment(cur_row, target.GetDOF()));\
        cur_row += _domains[i]->GetDOF();                           \
    }


void GroupDomainTarget::GetCoordinate(Ref<Eigen::VectorXd> x) const {
    ASSEMBLE_1D(Coordinate, x)
}

void GroupDomainTarget::GetVelocity(Ref<Eigen::VectorXd> v) const {
    ASSEMBLE_1D(Velocity, v)
}

#define DISPERSE_1D(FuncName, var)                                  \
    int cur_row = 0;                                                \
    for (int i = _begin; i < _end; i++) {                           \
        DomainTarget target(*_domains[i]);                          \
        target.Set##FuncName(var.segment(cur_row, target.GetDOF()));\
        cur_row += _domains[i]->GetDOF();                           \
    }

void GroupDomainTarget::SetCoordinate(const Ref<const Eigen::VectorXd> &x) {
    DISPERSE_1D(Coordinate, x)
}

void GroupDomainTarget::SetVelocity(const Ref<const Eigen::VectorXd> &v) {
    DISPERSE_1D(Velocity, v)
}

void GroupDomainTarget::GetMass(COO &coo, int offset_x, int offset_y) const {
    int current_row = 0;
    for (int i = _begin; i < _end; i++) {
        DomainTarget target(*_domains[i]);
        target.GetMass(coo, offset_x + current_row, offset_y + current_row);
        current_row += target.GetDOF();
    }
}

double GroupDomainTarget::GetPotentialEnergy() const {
    double energy = 0;
    for (int i = _begin; i < _end; i++) {
        DomainTarget target(*_domains[i]);
        energy += target.GetPotentialEnergy();
    }
    return energy;
}

double GroupDomainTarget::GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const {
    double energy = 0;
    int current_row = 0;
    for (int i = _begin; i < _end; i++) {
        DomainTarget target(*_domains[i]);
        energy += target.GetPotentialEnergy(x.segment(current_row, target.GetDOF()));
        current_row += target.GetDOF();
    }
    return energy;
}

void GroupDomainTarget::GetPotentialEnergyGradient(Ref<Eigen::VectorXd> gradient) const {
    int current_row = 0;
    for (int i = _begin; i < _end; i++) {
        DomainTarget target(*_domains[i]);
        target.GetPotentialEnergyGradient(gradient.segment(current_row, target.GetDOF()));
        current_row += target.GetDOF();
    }
}

void GroupDomainTarget::GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x,
                                                   Ref<Eigen::VectorXd> gradient) const {
    int current_row = 0;
    for (int i = _begin; i < _end; i++) {
        DomainTarget target(*_domains[i]);
        target.GetPotentialEnergyGradient(x.segment(current_row, target.GetDOF()), gradient.segment(current_row, target.GetDOF()));
        current_row += target.GetDOF();
    }
}

void GroupDomainTarget::GetPotentialEnergyHessian(COO &coo, int offset_x, int offset_y) const {
    int current_row = 0;
    for (int i = _begin; i < _end; i++) {
        DomainTarget target(*_domains[i]);
        target.GetPotentialEnergyHessian(coo, offset_x + current_row, offset_y + current_row);
        current_row += target.GetDOF();
    }
}

void GroupDomainTarget::GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x,
                                                  int offset_y) const {
    int current_row = 0;
    for (int i = _begin; i < _end; i++) {
        DomainTarget target(*_domains[i]);
        target.GetPotentialEnergyHessian(x.segment(current_row, target.GetDOF()), coo, offset_x + current_row, offset_y + current_row);
        current_row += target.GetDOF();
    }
}

void GroupDomainTarget::GetExternalForce(Ref<Eigen::VectorXd> force) const {
    int current_row = 0;
    for (int i = _begin; i < _end; i++) {
        DomainTarget target(*_domains[i]);
        target.GetExternalForce(force.segment(current_row, target.GetDOF()));
        current_row += target.GetDOF();
    }
}

VectorXd GroupDomainTarget::GetConstraint(const Eigen::VectorXd &x) const {
    // TODO
}

void GroupDomainTarget::GetConstraintGradient(SparseMatrixXd &gradient, const Eigen::VectorXd &x) const {
    // TODO
}