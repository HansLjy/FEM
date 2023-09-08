//
// Created by hansljy on 11/28/22.
//

#ifndef FEM_TIMESTEPPER_H
#define FEM_TIMESTEPPER_H

#include "System/System.hpp"

class TimeStepper {
public:
	TimeStepper() = default;
	TimeStepper(const TimeStepper& rhs) = delete;

	virtual void Bind(System* system) {
		_system = system;
	}

    virtual void Step(double h) = 0;
    virtual ~TimeStepper() = default;

protected:
    System* _system = nullptr;
};

#endif //FEM_TIMESTEPPER_H
