#pragma once

#include "Collision/CollisionInterface.hpp"
#include "Collision/CollisionInfo.hpp"
#include "SpatialQuery/SpatialHashing.hpp"

class BarrierSetGenerator {
public:
	static BarrierSetGenerator* GetProductFromConfig(const json& config);

	virtual void GenerateBarrierSet(
		const std::vector<CollisionInterface>& objs,
		const std::vector<int>& offsets,
		std::vector<PrimitivePair>& barrier_set
	) = 0;
};

class SpatialHashingBarrierSetGenerator : public BarrierSetGenerator {
public:
	static SpatialHashingBarrierSetGenerator* CreateFromConfig(const json& config);
	SpatialHashingBarrierSetGenerator(double d_hat, double grid_length, unsigned int hash_table_size)
		: _d_hat(d_hat),
		  _edge_hash_table(grid_length, hash_table_size),
		  _vertex_hash_table(grid_length, hash_table_size) {}

	void GenerateBarrierSet(
		const std::vector<CollisionInterface> &objs,
		const std::vector<int>& offsets,
		std::vector<PrimitivePair> &barrier_set
	) override;

public:
	double _d_hat;

	int _time_stamp = 0;
	SpatialHashing<EdgePrimitiveInfo> _edge_hash_table;
	SpatialHashing<VertexPrimitiveInfo> _vertex_hash_table;
};
