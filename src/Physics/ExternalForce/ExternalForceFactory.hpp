#pragma once

#include "ExternalForce.hpp"
#include "SampledObjectGravity.h"

template <class Object>
class ExternalForceFactory {
public:
    using ExternalForceGenerator = std::function<ExternalForce<Object>*(const json& config)>;

    ExternalForceFactory<Object>() {
        if (SampledObjectGravity<Object>::valid) {
            _generators.insert(std::make_pair("sampled-object-gravity", [](const json& config) {return new SampledObjectGravity<Object>(config);}));
        }
    }

    static ExternalForceFactory<Object>* Instance() {
        if (_the_factory == nullptr) {
            _the_factory = new ExternalForceFactory<Object>;
        }
        return _the_factory;
    }

    ExternalForce<Object>* GetExternalForce(const std::string& name, const json& config) {
        return _generators[name](config);
    }

protected:
    static ExternalForceFactory<Object>* _the_factory;
    std::map<std::string, ExternalForceGenerator> _generators;
};

template<class Object>
ExternalForceFactory<Object>* ExternalForceFactory<Object>::_the_factory = nullptr;