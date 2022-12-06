//
// Created by hansljy on 11/29/22.
//

#ifndef FEM_DOMAINBFSSTEPPER_H
#define FEM_DOMAINBFSSTEPPER_H

#include "TimeStepper.h"
#include "Integrator/Integrator.h"
#include "Domain.h"

class DomainBFSStepper : public TimeStepper {
public:
    explicit DomainBFSStepper(const json& config);
    void Bind(System &system) override;
    void Step(double h) const override;

    ~DomainBFSStepper() noexcept override;

protected:
    Integrator* _integrator;
    std::vector<Domain*> _domains;
    std::vector<Target*> _level_targets;
    std::vector<int> _level_bar;
    int _level;
};

class GroupDomainIterator : public ObjectIterator {
public:
    GroupDomainIterator(const std::vector<Domain*>& domains, int begin, int end)
        : ObjectIterator(begin == end), _domains(domains), _begin(begin), _end(end),
          _cur_itr(new SystemIterator(*domains[begin])), _cur_domain(begin) {}

    void Forward() override;
    Object * GetObject() override;

    std::shared_ptr<ObjectIterator> Clone() const override {
        return std::shared_ptr<ObjectIterator>(new GroupDomainIterator(*this));
    }

protected:
    const std::vector<Domain*>& _domains;
    const int _begin, _end;
    std::shared_ptr<ObjectIterator> _cur_itr;
    int _cur_domain;
};

#endif //FEM_DOMAINBFSSTEPPER_H
