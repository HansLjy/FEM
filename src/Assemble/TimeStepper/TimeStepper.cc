#include "TimeStepper.hpp"
#include "Pattern.h"

template<>
Factory<TimeStepper>* Factory<TimeStepper>::_the_factory = nullptr;

// #include "IncrementalPotential.hpp"
// #include "ProjectiveDynamics.hpp"
// #include "PDIPC.hpp"
#include "PositionBasedPDIPC.hpp"

// const bool ip_registered = FactoryRegistration::RegisterForFactory<TimeStepper, IncrementalPotentialTimeStepper>("incremental-potential");
// const bool pd_registerd = FactoryRegistration::RegisterForFactory<TimeStepper, ProjectiveDynamics>("projective-dynamics");
// const bool pdipc_registered = FactoryRegistration::RegisterForFactory<TimeStepper, PDIPC>("PDIPC");
const bool position_based_pdipc_registered = FactoryRegistration::RegisterForFactory<TimeStepper, PositionBasedPDIPC>("position-based-PDIPC");

void TimeStepperUtils::DumpXV(int itr, const VectorXd& x, const VectorXd& v) {
	std::ofstream x_file(std::string(OUTPUT_PATH) + "/dump/x" + std::to_string(itr));
	std::ofstream v_file(std::string(OUTPUT_PATH) + "/dump/v" + std::to_string(itr));
	write_binary(x_file, x);
	write_binary(v_file, v);
}

void TimeStepperUtils::PickXV(int itr, VectorXd &x, VectorXd &v) {
	std::ifstream x_file(std::string(OUTPUT_PATH) + "/dump/x" + std::to_string(itr));
	std::ifstream v_file(std::string(OUTPUT_PATH) + "/dump/v" + std::to_string(itr));
	read_binary(x_file, x);
	read_binary(v_file, v);
}