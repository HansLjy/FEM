#pragma once

#include "Object.hpp"

struct System {
	System() = default;
    System(const System& rhs) = delete;
	virtual std::vector<Object*>& GetObjs() = 0;
    virtual ~System() = default;
};