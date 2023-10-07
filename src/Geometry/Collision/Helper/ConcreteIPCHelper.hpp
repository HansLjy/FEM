#pragma once

#include "IPCHelper.hpp"
#include "ConstraintSetGenerator.hpp"
#include "MaxStepEstimator.hpp"

using OriginalIPCHelper = ConcreteIPCHelper<NormalConstraintSetGenerator, NormalMaxStepEstimator>;