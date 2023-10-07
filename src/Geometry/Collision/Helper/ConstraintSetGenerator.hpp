#pragma once

#include "IPCHelper.hpp"
#include "Collision/SpatialHashing/SpatialHashing.hpp"

class NormalConstraintSetGenerator {
public:
	void ComputeConstraintSet(
		const VectorXd &x,
		double d_hat,
		const std::vector<CollisionShapeInterface*>& objs,
		const std::vector<int>& offsets,
		const std::vector<int>& dofs,
		std::vector<CollisionInfo>& constraint_set
	);

	int _time_stamp = 0;
	SpatialHashing<EdgePrimitiveInfo> _edge_hash_table;
	SpatialHashing<VertexPrimitiveInfo> _vertex_hash_table;
};