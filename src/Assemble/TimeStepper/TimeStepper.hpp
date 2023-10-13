#pragma once

#include "Object.hpp"
#include "Pattern.h"

class TimeStepper {
public:
	TimeStepper() = default;
	TimeStepper(const TimeStepper& rhs) = delete;

	virtual void BindObjects(
		const typename std::vector<Object>::const_iterator& begin,
		const typename std::vector<Object>::const_iterator& end
	) = 0;
    virtual void Step(double h) = 0;
    virtual ~TimeStepper() = default;
};

namespace TimeStepperRegistration {
	template<class T>
	bool RegisterTimeStepper(const std::string& type) {
		return Factory<TimeStepper>::GetInstance()->Register(type, [](const json& config){
			return new T(config);
		});
	}
}
