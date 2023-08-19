//
// Created by hansljy on 11/28/22.
//

#ifndef FEM_TIMESTEPPER_H
#define FEM_TIMESTEPPER_H

#include "System.h"

class TimeStepper {
public:
    explicit TimeStepper(const json& config)
        : _target_config(config["target"]) {}
    
	virtual void Bind(System& system);
    virtual void Step(double h) const = 0;
    virtual ~TimeStepper() = default;
	TimeStepper(const TimeStepper& rhs) = delete;

protected:
    System* _system = nullptr;
    json _target_config;
};

#endif //FEM_TIMESTEPPER_H
