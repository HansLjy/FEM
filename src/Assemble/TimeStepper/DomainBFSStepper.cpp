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
        }
    }

    // TODO: generate targets
}

void DomainBFSStepper::Step(double h) const {
    // TODO
}

DomainBFSStepper::~DomainBFSStepper() noexcept {
    delete _integrator;
}