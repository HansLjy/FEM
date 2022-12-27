//
// Created by hansljy on 11/28/22.
//

#ifndef FEM_TIMESTEPPER_H
#define FEM_TIMESTEPPER_H

#include "System.h"

class TimeStepper {
public:
    explicit TimeStepper(const json& config)
        : _collision_aware(config["collision-aware"]), _collision_config(config["collision"]){}
    
	virtual void Bind(System& system);
    virtual void Step(double h) const = 0;
    virtual ~TimeStepper() = default;
	TimeStepper(const TimeStepper& rhs) = delete;

protected:
    System* _system = nullptr;
    bool _collision_aware;
    json _collision_config;
};

DECLARE_XXX_FACTORY(TimeStepper)

#endif //FEM_TIMESTEPPER_H
