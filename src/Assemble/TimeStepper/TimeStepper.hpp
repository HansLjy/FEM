#pragma once

#include "Object.hpp"
#include "Pattern.h"
#include "Render/RenderInterface.hpp"

class TimeStepper {
public:
	TimeStepper() = default;
	TimeStepper(const TimeStepper& rhs) = delete;

	virtual void BindSystem(const json& config) = 0;
    virtual void Step(double h) = 0;
	virtual const std::vector<Renderable>& GetRenderObjects() const = 0;
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
