#include "CCDCulling.hpp"
#include "Pattern.h"

template<>
Factory<CCDCulling>* Factory<CCDCulling>::_the_factory = nullptr;

CCDCulling* CCDCulling::GetProductFromConfig(const json &config) {
	return Factory<CCDCulling>::GetInstance()->GetProduct(config["type"], config);
}

#include "SpatialHashingCulling.hpp"

const bool sphs_culling_registered = FactoryRegistration::RegisterForFactory<CCDCulling, SpatialHashingCulling>("spatial-hashing");