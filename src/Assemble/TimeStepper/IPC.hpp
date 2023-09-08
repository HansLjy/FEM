#pragma once

#include "TimeStepper.hpp"
#include "Assembler/Assembler.hpp"
#include "Collision/Helper/IPCHelper.hpp"

class IPC : public TimeStepper {
public:
	explicit IPC(const json& config);

	void Step(double h);
	~IPC();
	
	IPCHelper* _helper = nullptr;

private:
	Assembler* _assembler = nullptr;
	int _max_iter;
    double _tolerance;
};
