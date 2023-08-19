//
// Created by hansljy on 11/28/22.
//

#include "TimeStepper.h"

template<>
Factory<TimeStepper>* Factory<TimeStepper>::_the_factory = nullptr;

void TimeStepper::Bind(System &system) {
    _system = &system;
}