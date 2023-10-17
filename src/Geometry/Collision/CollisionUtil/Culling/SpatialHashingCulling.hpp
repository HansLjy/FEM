#pragma once

#include "CCDCulling.hpp"
#include "SpatialQuery/SpatialHashing.hpp"

class SpatialHashingCulling : public CCDCulling {
public:
	explicit SpatialHashingCulling(const json& config);
	void GetCCDSet(const std::vector<CollisionInterface> &objs, const std::vector<int> &offsets, std::vector<PrimitivePair> &ccd_set) override;

protected:
	SpatialHashing<EdgePrimitiveInfo> _edge_hash_table;
	SpatialHashing<VertexPrimitiveInfo> _vertex_hash_table;

	unsigned int _time_stamp = 0;
};