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
	virtual void DumpCoord(int itr) const {}	// TODO:
	virtual void PickCoord(int itr) {};			// TODO:
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

#include "Assembler/Assembler.hpp"

namespace TimeStepperUtils {
	void DumpXV(int itr, const VectorXd& x, const VectorXd& v);
	void PickXV(int itr, VectorXd& x, VectorXd& v);
}
