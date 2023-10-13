#pragma once

#include "Assembler/Assembler.hpp"
#include "Assembler/PDAssembler.hpp"
#include "TimeStepper.hpp"

class ProjectiveDynamics : public TimeStepper {
public:
    ProjectiveDynamics(const json& config);

    void BindObjects(
        const typename std::vector<Object>::const_iterator &begin,
        const typename std::vector<Object>::const_iterator &end
    ) override;
    
    void Step(double h) override;

protected:
    int _max_step;
    double _conv_tolerance;

    CoordinateAssembler _coord_assembler;
    MassAssembler _mass_assembler;
    ExternalForceAssembler _ext_force_assembler;
    PDAssembler _pd_assembler;
};

namespace ObjectRegistration {
    template<class T>
    bool RegisterForPD(const std::string& type) {
        CasterRegistration::RegisterForCaster<Coordinated, T>(type);
        CasterRegistration::RegisterForCaster<Massed, T>(type);
        CasterRegistration::RegisterForCaster<ExternalForced, T>(type);
        CasterRegistration::RegisterForCaster<PDObject, T>(type);
        return true;
    }
}
