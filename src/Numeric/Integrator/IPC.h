//
// Created by hansljy on 12/5/22.
//

#ifndef FEM_IPC_H
#define FEM_IPC_H

#include "Integrator.h"

class IPC : public Integrator {
public:
    explicit IPC(const json& config) : Integrator(config), _max_iter(config["max-iteration"]), _tolerance(config["tolerance"]) {};
    void Step(Target &target, double h) const override;

protected:
    int _max_iter;
    double _tolerance;
};

#endif //FEM_IPC_H
