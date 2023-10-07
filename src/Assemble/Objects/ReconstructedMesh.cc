#include "ReconstructedMesh.hpp"

template<>
Factory<ExternalForce<ReconstructedMeshData>>* Factory<ExternalForce<ReconstructedMeshData>>::_the_factory = nullptr;

namespace {
	const bool rm_registed = Factory<Object>::GetInstance()->Register("reconstructed-mesh",
		[](const json& config) {
			return new ReconstructedMesh(config);
		}
	);
}